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
  
  if (!this->connect()) {
      RCLCPP_ERROR(this->get_logger(), "Initial connection failed. Activating reconnect timer.");
      // Use the faster timer period here as well
      reconnect_timer_ = this->create_wall_timer(500ms, std::bind(&HikCameraNode::connect, this));
  }
}

HikCameraNode::~HikCameraNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down HikCameraNode...");
  if (grab_thread_.joinable()) {
      is_grabbing_ = false;
      grab_thread_.join();
  }
  disconnect();
}

void HikCameraNode::declare_ros_parameters()
{
  this->declare_parameter<std::string>("camera_sn", "");
  this->declare_parameter<std::string>("camera_info_url", "");
  this->declare_parameter<double>("exposure_time", 5000.0);
  this->declare_parameter<double>("gain", 10.0);
  this->declare_parameter<double>("frame_rate", 20.0);

  this->get_parameter("camera_sn", camera_sn_);
  this->get_parameter("camera_info_url", camera_info_url_);
  this->get_parameter("exposure_time", exposure_time_);
  this->get_parameter("gain", gain_);
  this->get_parameter("frame_rate", frame_rate_);
}

bool HikCameraNode::connect()
{
  if (grab_thread_.joinable()) {
    is_grabbing_ = false;
    grab_thread_.join();
    RCLCPP_INFO(this->get_logger(), "Old grab thread joined.");
  }
  
  this->disconnect();
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(this->get_logger(), "Attempting to connect to camera...");
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet || stDeviceList.nDeviceNum == 0) {
    RCLCPP_ERROR(this->get_logger(), "No cameras found or enum devices fail.");
    return false;
  }

  int device_index = -1;
  for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
    if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE) {
        std::string sn = (char*)stDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber;
        if (camera_sn_.empty() || sn == camera_sn_) {
            device_index = i;
            break;
        }
    }
  }

  if (device_index < 0) {
    RCLCPP_ERROR(this->get_logger(), "Target USB camera with SN [%s] not found.", camera_sn_.c_str());
    return false;
  }
  nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[device_index]);
  if (MV_OK != nRet) { return false; }
  nRet = MV_CC_OpenDevice(handle_);
  if (MV_OK != nRet) {
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }
  
  if (reconnect_timer_ && !reconnect_timer_->is_canceled()) {
    reconnect_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Reconnection successful, stopping timer.");
  }

  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam);
  if (nRet != MV_OK) { this->disconnect(); return false; }
  n_data_size_ = stParam.nCurValue;
  p_data_buffer_ = std::make_unique<unsigned char[]>(n_data_size_);
  if (p_data_buffer_ == nullptr) { this->disconnect(); return false; }
  
  this->apply_all_parameters();

  nRet = MV_CC_StartGrabbing(handle_);
  if (MV_OK != nRet) { this->disconnect(); return false; }

  is_grabbing_ = true;
  grab_thread_ = std::thread(&HikCameraNode::grab_loop, this);
  RCLCPP_INFO(this->get_logger(), "Camera connected successfully, grab thread started.");
  return true;
}

void HikCameraNode::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_grabbing_ = false;
  if (handle_ != nullptr)
  {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }
  p_data_buffer_.reset();
  n_data_size_ = 0;
}

void HikCameraNode::grab_loop()
{
  MV_FRAME_OUT_INFO_EX stImageInfo;
  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  while (is_grabbing_ && rclcpp::ok()) {
    int nRet = MV_CC_GetOneFrameTimeout(handle_, p_data_buffer_.get(), n_data_size_, &stImageInfo, 1000);
    if (nRet == MV_OK) {
      if (stImageInfo.nFrameLen > 0) {
        sensor_msgs::msg::Image ros_image;
        if(this->convert_to_ros_image(p_data_buffer_.get(), &stImageInfo, ros_image)) {
            sensor_msgs::msg::CameraInfo camera_info_msg = camera_info_manager_->getCameraInfo();
            ros_image.header.stamp = this->now();
            ros_image.header.frame_id = "camera_frame";
            camera_info_msg.header = ros_image.header;
            image_pub_.publish(ros_image, camera_info_msg);
        }
      }
    } else {
      if ((unsigned int)nRet != MV_E_GC_TIMEOUT) {
        RCLCPP_ERROR(this->get_logger(), "Grab image failed! Critical error: [0x%x]. Connection lost.", nRet);
        is_grabbing_ = false;
        if (!reconnect_timer_) {
            reconnect_timer_ = this->create_wall_timer(2s, std::bind(&HikCameraNode::connect, this));
        } else {
            reconnect_timer_->reset();
        }
        RCLCPP_INFO(this->get_logger(), "Reconnect timer (2s) activated.");
        return;
      }
    }
  }
  RCLCPP_INFO(this->get_logger(), "Grab thread stopped gracefully.");
}

bool HikCameraNode::apply_all_parameters()
{
  if (handle_ == nullptr) return false;
  int nRet;
  bool all_success = true;
  nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_time_);
  if(nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set exposure time. Error code: [0x%x]", nRet);
      all_success = false;
  }
  nRet = MV_CC_SetFloatValue(handle_, "Gain", gain_);
  if(nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set gain. Error code: [0x%x]", nRet);
      all_success = false;
  }
  nRet = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", frame_rate_);
  if(nRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set frame rate. Error code: [0x%x]", nRet);
      all_success = false;
  }
  return all_success;
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

bool HikCameraNode::convert_to_ros_image(
    unsigned char * pData, 
    MV_FRAME_OUT_INFO_EX * pFrameInfo, 
    sensor_msgs::msg::Image & ros_image)
{
  MvGvspPixelType enPixelType = pFrameInfo->enPixelType;

  if (enPixelType == PixelType_Gvsp_Mono8) 
  {
    cv::Mat cv_image = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cv_image).toImageMsg(ros_image);
    return true;
  } 
  else if (enPixelType == PixelType_Gvsp_BayerRG8) 
  {
    cv::Mat bayer_image = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
    cv::Mat color_image;
    cv::cvtColor(bayer_image, color_image, cv::COLOR_BayerRG2RGB);
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_image).toImageMsg(ros_image);
    return true;
  } 
  else 
  {
    RCLCPP_WARN_ONCE(this->get_logger(), "Unsupported pixel format [0x%x]. Cannot convert to ROS image.", (unsigned int)enPixelType);
    return false;
  }
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