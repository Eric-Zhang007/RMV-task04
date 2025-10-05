#include "hik_camera_ros2/hik_camera_node.hpp"
#include <chrono>

namespace hikcam
{

using namespace std::chrono_literals;
using std::placeholders::_1;

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions & options)
: Node("hik_camera_node", options),
  is_grabbing_(false)
{
  RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode...");

  this->declare_ros_parameters();

  camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
  if (camera_info_manager_->loadCameraInfo(camera_info_url_)) {
    RCLCPP_INFO(this->get_logger(), "Loaded camera calibration from %s", camera_info_url_.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "Could not load camera calibration from %s", camera_info_url_.c_str());
  }

  image_pub_ = image_transport::create_camera_publisher(this, "image_raw");

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&HikCameraNode::parameters_callback, this, _1));

  if (this->connect()) {
    RCLCPP_INFO(this->get_logger(), "Camera connected successfully.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to camera. Will retry every 2 seconds.");
    reconnect_timer_ = this->create_wall_timer(2s, std::bind(&HikCameraNode::connect, this));
  }
}

HikCameraNode::~HikCameraNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down HikCameraNode...");
  this->disconnect();
}

void HikCameraNode::declare_ros_parameters()
{
  this->declare_parameter<std::string>("camera_ip", "192.168.1.100");
  this->declare_parameter<std::string>("camera_info_url", "");
  this->declare_parameter<double>("exposure_time", 2000.0);
  this->declare_parameter<double>("gain", 5.0);
  this->declare_parameter<double>("frame_rate", 20.0);

  this->get_parameter("camera_ip", camera_ip_);
  this->get_parameter("camera_info_url", camera_info_url_);
  this->get_parameter("exposure_time", exposure_time_);
  this->get_parameter("gain", gain_);
  this->get_parameter("frame_rate", frame_rate_);
}

bool HikCameraNode::connect()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (handle_ != nullptr) {
    return true;
  }

  RCLCPP_INFO(this->get_logger(), "Attempting to connect to camera at IP: %s", camera_ip_.c_str());

  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_EnumDevices fail! nRet [0x%x]", nRet);
    return false;
  }

  if (stDeviceList.nDeviceNum == 0) {
    RCLCPP_ERROR(this->get_logger(), "No cameras found.");
    return false;
  }

  int device_index = -1;
  for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
    MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
    if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
      std::string ip_from_device = std::to_string((pDeviceInfo->SpecialInfo.stGigEInfo.nIpCfgCurrent & 0xFF000000) >> 24) + "." +
                                    std::to_string((pDeviceInfo->SpecialInfo.stGigEInfo.nIpCfgCurrent & 0x00FF0000) >> 16) + "." +
                                    std::to_string((pDeviceInfo->SpecialInfo.stGigEInfo.nIpCfgCurrent & 0x0000FF00) >> 8) + "." +
                                    std::to_string(pDeviceInfo->SpecialInfo.stGigEInfo.nIpCfgCurrent & 0x000000FF);
      if (ip_from_device == camera_ip_) {
        device_index = i;
        break;
      }
    }
  }

  if (device_index < 0) {
    RCLCPP_ERROR(this->get_logger(), "Camera with IP %s not found.", camera_ip_.c_str());
    return false;
  }

  nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[device_index]);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_CreateHandle fail! nRet [0x%x]", nRet);
    return false;
  }

  nRet = MV_CC_OpenDevice(handle_);
  if (MV_OK != nRet) {
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    RCLCPP_ERROR(this->get_logger(), "MV_CC_OpenDevice fail! nRet [0x%x]", nRet);
    return false;
  }

  // === NEW: Get payload size and allocate buffer ===
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam);
  if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Get PayloadSize fail! nRet [0x%x]", nRet);
      return false;
  }
  n_data_size_ = stParam.nCurValue;
  p_data_buffer_ = std::make_unique<unsigned char[]>(n_data_size_);
  if (p_data_buffer_ == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for image buffer.");
      return false;
  }
  RCLCPP_INFO(this->get_logger(), "Image buffer allocated with size: %d bytes.", n_data_size_);

  if (!this->apply_all_parameters()) {
     RCLCPP_ERROR(this->get_logger(), "Failed to apply initial camera parameters.");
  }

  nRet = MV_CC_StartGrabbing(handle_);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_StartGrabbing fail! nRet [0x%x]", nRet);
    return false;
  }

  is_grabbing_ = true;
  grab_thread_ = std::thread(&HikCameraNode::grab_loop, this);

  if (reconnect_timer_) {
    reconnect_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Reconnection successful, stopping timer.");
  }

  return true;
}

void HikCameraNode::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (handle_ == nullptr) {
    return;
  }

  is_grabbing_ = false;
  if (grab_thread_.joinable()) {
    grab_thread_.join();
  }

  MV_CC_StopGrabbing(handle_);
  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);
  handle_ = nullptr;
  p_data_buffer_.reset(); // Release buffer memory
  n_data_size_ = 0;

  RCLCPP_INFO(this->get_logger(), "Camera disconnected.");
}

bool HikCameraNode::apply_all_parameters()
{
  if (handle_ == nullptr) return false;
  int nRet;
  nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_time_);
  if(nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set exposure time. Error code: [0x%x]", nRet);
  }
  nRet = MV_CC_SetFloatValue(handle_, "Gain", gain_);
  if(nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set gain. Error code: [0x%x]", nRet);
  }
  nRet = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", frame_rate_);
  if(nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set frame rate. Error code: [0x%x]", nRet);
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    result.successful = false;
    result.reason = "Camera not connected.";
    return result;
  }

  for (const auto & param : parameters) {
    if (param.get_name() == "exposure_time") {
      exposure_time_ = param.as_double();
      MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_time_);
    } else if (param.get_name() == "gain") {
      gain_ = param.as_double();
      MV_CC_SetFloatValue(handle_, "Gain", gain_);
    } else if (param.get_name() == "frame_rate") {
      frame_rate_ = param.as_double();
      MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", frame_rate_);
    }
  }

  return result;
}

void HikCameraNode::grab_loop()
{
  // === NEW: Correct data structures for the new API ===
  MV_FRAME_OUT_INFO_EX stImageInfo;
  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  sensor_msgs::msg::Image ros_image;

  while (is_grabbing_ && rclcpp::ok()) {
    // === NEW: Call the function with the correct signature ===
    int nRet = MV_CC_GetOneFrameTimeout(handle_, p_data_buffer_.get(), n_data_size_, &stImageInfo, 1000);
    if (nRet == MV_OK) {
      if (stImageInfo.nFrameLen > 0) {
        if(this->convert_to_ros_image(p_data_buffer_.get(), &stImageInfo, ros_image)) {
            sensor_msgs::msg::CameraInfo camera_info_msg = camera_info_manager_->getCameraInfo();
            ros_image.header.stamp = this->now();
            ros_image.header.frame_id = "camera_frame";
            camera_info_msg.header = ros_image.header;
            image_pub_.publish(ros_image, camera_info_msg);
        }
      }
    } else {
        // === FIXED: Correct timeout error code ===
        if ((unsigned int)nRet != MV_E_GC_TIMEOUT) {
            RCLCPP_WARN(this->get_logger(), "MV_CC_GetOneFrameTimeout fail! nRet [0x%x]", nRet);
        }
    }
  }
}

// === NEW: Updated function to match the new API ===
bool HikCameraNode::convert_to_ros_image(
    unsigned char * pData, 
    MV_FRAME_OUT_INFO_EX * pFrameInfo, 
    sensor_msgs::msg::Image & ros_image)
{
  cv::Mat cv_image;
  MvGvspPixelType enPixelType = pFrameInfo->enPixelType;

  if (enPixelType == PixelType_Gvsp_Mono8) {
    cv_image = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cv_image).toImageMsg(ros_image);
  } else if (enPixelType == PixelType_Gvsp_BayerRG8) {
    cv_image = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
    cv_bridge::CvImage(std_msgs::msg::Header(), "bayer_rg8", cv_image).toImageMsg(ros_image);
  } else {
    RCLCPP_WARN_ONCE(this->get_logger(), "Unsupported pixel format [0x%x]. Only Mono8 and BayerRG8 are currently supported.", (unsigned int) enPixelType);
    return false;
  }
  
  return true;
}

} // namespace hikcam

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<hikcam::HikCameraNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}