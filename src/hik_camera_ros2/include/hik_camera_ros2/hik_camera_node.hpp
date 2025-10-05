#ifndef HIK_CAMERA_NODE_HPP_
#define HIK_CAMERA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "cv_bridge/cv_bridge.h"
#include <thread>
#include <mutex>
#include <memory> // Required for std::unique_ptr

#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"

namespace hikcam
{

class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options);
  ~HikCameraNode();

private:
  image_transport::CameraPublisher image_pub_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  void* handle_ = nullptr;
  std::thread grab_thread_;
  std::atomic<bool> is_grabbing_;
  std::mutex mutex_;

  // === NEW: Buffer for image data ===
  // 使用智能指针管理内存，防止泄露
  std::unique_ptr<unsigned char[]> p_data_buffer_ = nullptr;
  unsigned int n_data_size_ = 0;

  std::string camera_sn_ = "";
  std::string camera_info_url_ = "";
  double exposure_time_ = 2000.0;
  double gain_ = 5.0;
  double frame_rate_ = 20.0;

  bool connect();
  void disconnect();
  void grab_loop();
  void declare_ros_parameters();
  bool apply_all_parameters();
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);
  
  // === NEW: Updated function signature ===
  bool convert_to_ros_image(
    unsigned char * pData, 
    MV_FRAME_OUT_INFO_EX * pFrameInfo, 
    sensor_msgs::msg::Image & ros_image);
};

}  // namespace hikcam

#endif  // HIK_CAMERA_NODE_HPP_