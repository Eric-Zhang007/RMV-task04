#include "hik_camera_ros2/hik_camera_node.hpp"
#include <chrono>

namespace hikcam
{

using namespace std::chrono_literals;
using std::placeholders::_1;


MvGvspPixelType string_to_pixel_format(const std::string& format_str)
{
    if (format_str == "Mono8") return PixelType_Gvsp_Mono8;
    if (format_str == "BayerRG8") return PixelType_Gvsp_BayerRG8;
    if (format_str == "RGB8") return PixelType_Gvsp_RGB8_Packed;
    if (format_str == "BGR8") return PixelType_Gvsp_BGR8_Packed;
    if (format_str == "Mono10") return PixelType_Gvsp_Mono10;
    if (format_str == "Mono12") return PixelType_Gvsp_Mono12;
    if (format_str == "BayerRG10") return PixelType_Gvsp_BayerRG10;
    if (format_str == "BayerRG12") return PixelType_Gvsp_BayerRG12;
    if (format_str == "YUV422_Packed") return PixelType_Gvsp_YUV422_Packed;
    return PixelType_Gvsp_Undefined;
}

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
  frame_rate_pub_ = this->create_publisher<std_msgs::msg::Float32>("actual_frame_rate", 10);
  frame_rate_timer_ = this->create_wall_timer(1s, std::bind(&HikCameraNode::publish_actual_frame_rate, this));
  last_frame_rate_time_ = this->now();
  if (!this->connect()) {
      RCLCPP_ERROR(this->get_logger(), "Initial connection failed. Activating reconnect timer.");
      reconnect_timer_ = this->create_wall_timer(2s, std::bind(&HikCameraNode::connect, this));
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
  this->declare_parameter<std::string>("pixel_format", "BayerRG8");

  this->get_parameter("camera_sn", camera_sn_);
  this->get_parameter("camera_info_url", camera_info_url_);
  this->get_parameter("exposure_time", exposure_time_);
  this->get_parameter("gain", gain_);
  this->get_parameter("frame_rate", frame_rate_);
  this->get_parameter("pixel_format", pixel_format_); 
  this->declare_parameter<int>("roi_width", -1);
  this->declare_parameter<int>("roi_height", -1);
  this->declare_parameter<int>("roi_offset_x", -1);
  this->declare_parameter<int>("roi_offset_y", -1);
  this->get_parameter("roi_width", roi_width_);
  this->get_parameter("roi_height", roi_height_);
  this->get_parameter("roi_offset_x", roi_offset_x_);
  this->get_parameter("roi_offset_y", roi_offset_y_);
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
    RCLCPP_WARN(this->get_logger(), "No cameras found or enum devices fail.");
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
    RCLCPP_WARN(this->get_logger(), "Target USB camera with SN [%s] not found.", camera_sn_.c_str());
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

  if (!this->apply_all_parameters()) {
     RCLCPP_WARN(this->get_logger(), "Failed to apply one or more initial camera parameters.");
  }

  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam);
  if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Get PayloadSize fail! nRet [0x%x]", nRet);
      this->disconnect();
      return false;
  }
  n_data_size_ = stParam.nCurValue;
  p_data_buffer_ = std::make_unique<unsigned char[]>(n_data_size_);
  if (p_data_buffer_ == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for image buffer.");
      this->disconnect();
      return false;
  }
  RCLCPP_INFO(this->get_logger(), "Image buffer for format '%s' allocated with size: %d bytes.", pixel_format_.c_str(), n_data_size_);

  nRet = MV_CC_StartGrabbing(handle_);
  if (MV_OK != nRet) {
      RCLCPP_ERROR(this->get_logger(), "MV_CC_StartGrabbing fail! nRet [0x%x]", nRet);
      this->disconnect();
      return false;
  }

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
            frame_counter_++;
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

  MvGvspPixelType pixel_format_enum = string_to_pixel_format(pixel_format_);
  if (pixel_format_enum == PixelType_Gvsp_Undefined) {
      RCLCPP_WARN(this->get_logger(), "Unsupported pixel format string: %s", pixel_format_.c_str());
      all_success = false;
  } else {
      nRet = MV_CC_SetEnumValue(handle_, "PixelFormat", pixel_format_enum);
      if(nRet != MV_OK) {
          RCLCPP_WARN(this->get_logger(), "Failed to set pixel format to %s. Error code: [0x%x]", pixel_format_.c_str(), nRet);
          all_success = false;
      }
  }

  if (roi_offset_x_ != -1) {
  // Note: The parameter name for MVS SDK is "OffsetX"
  nRet = MV_CC_SetIntValue(handle_, "OffsetX", roi_offset_x_);
    if(nRet != MV_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to set OffsetX. Error: [0x%x]", nRet);
        all_success = false;
    }
  }
  if (roi_offset_y_ != -1) {
    nRet = MV_CC_SetIntValue(handle_, "OffsetY", roi_offset_y_);
    if(nRet != MV_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to set OffsetY. Error: [0x%x]", nRet);
        all_success = false;
    }
  }
  if (roi_width_ != -1) {
    nRet = MV_CC_SetIntValue(handle_, "Width", roi_width_);
    if(nRet != MV_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to set Width. Error: [0x%x]", nRet);
        all_success = false;
    }
  }
  if (roi_height_ != -1) {
    nRet = MV_CC_SetIntValue(handle_, "Height", roi_height_);
    if(nRet != MV_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to set Height. Error: [0x%x]", nRet);
        all_success = false;
    }
  }

  nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_time_);
  if(nRet != MV_OK) { all_success = false; }
  nRet = MV_CC_SetFloatValue(handle_, "Gain", gain_);
  if(nRet != MV_OK) { all_success = false; }
  nRet = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", frame_rate_);
  if(nRet != MV_OK) { all_success = false; }
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

  if (enPixelType == PixelType_Gvsp_Mono8) {
    cv::Mat cv_image = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cv_image).toImageMsg(ros_image);
    return true;
  } 
  else if (enPixelType == PixelType_Gvsp_BayerRG8) {
    cv::Mat bayer_image = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
    cv::Mat color_image;
    cv::cvtColor(bayer_image, color_image, cv::COLOR_BayerRG2RGB);
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_image).toImageMsg(ros_image);
    return true;
  }
  else if (enPixelType == PixelType_Gvsp_RGB8_Packed) {
    cv::Mat cv_image = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, pData);
    cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", cv_image).toImageMsg(ros_image);
    return true;
  }
  else if (enPixelType == PixelType_Gvsp_BGR8_Packed) {
    cv::Mat cv_image = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, pData);
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg(ros_image);
    return true;
  }
  else if (enPixelType == PixelType_Gvsp_Mono10 || enPixelType == PixelType_Gvsp_Mono12) {
    cv::Mat cv_image = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_16UC1, pData);
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", cv_image).toImageMsg(ros_image);
    return true;
  }
  else if (enPixelType == PixelType_Gvsp_BayerRG10 || enPixelType == PixelType_Gvsp_BayerRG12) {
    cv::Mat cv_image = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_16UC1, pData);
    cv_bridge::CvImage(std_msgs::msg::Header(), "bayer_rg16", cv_image).toImageMsg(ros_image);
    return true;
  }
  else if (enPixelType == PixelType_Gvsp_YUV422_Packed) {
    cv::Mat yuv_image = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC2, pData);
    cv::Mat color_image;
    cv::cvtColor(yuv_image, color_image, cv::COLOR_YUV2RGB_YUY2); // YUY2 is the common name for this packed format
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_image).toImageMsg(ros_image);
    return true;
  }
  else {
    // CORRECTED: The format specifier for a long int (which enPixelType is) should be %lx
    RCLCPP_WARN_ONCE(this->get_logger(), "Unsupported pixel format [0x%lx]. Cannot convert to ROS image.", enPixelType);
    return false;
  }
}

/*void HikCameraNode::publish_actual_frame_rate()
{
    if (handle_ == nullptr || !is_grabbing_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    MVCC_FLOATVALUE stFloatValue;
    memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));
    int nRet = MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &stFloatValue);

    if (nRet == MV_OK)
    {
        auto msg = std_msgs::msg::Float32();
        msg.data = stFloatValue.fCurValue;
        frame_rate_pub_->publish(msg);
    }
    else
    {
        RCLCPP_WARN_ONCE(
            this->get_logger(), 
            "Failed to get actual frame rate from camera. Error code: [0x%x]", nRet);
    }
}*/
void HikCameraNode::publish_actual_frame_rate()
{
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_frame_rate_time_).seconds();
    int frame_count = frame_counter_.exchange(0);
    if (dt > 0.0)
    {
        double frame_rate = static_cast<double>(frame_count) / dt;
        auto msg = std_msgs::msg::Float32();
        msg.data = frame_rate;
        RCLCPP_INFO(this->get_logger(), "Publishing actual frame rate: %.2f", frame_rate);
        frame_rate_pub_->publish(msg);
    }
    last_frame_rate_time_ = current_time;
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
