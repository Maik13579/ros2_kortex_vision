#pragma once

#include <atomic>
#include <memory>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <rclcpp/rclcpp.hpp>

#include <camera_info_manager/camera_info_manager.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

namespace ros_kortex_vision
{
class VisionComponent : public rclcpp::Node
{
public:
  explicit VisionComponent(const rclcpp::NodeOptions& options);
  ~VisionComponent() override;
  void quit();

private:
  bool configure();
  bool initialize();
  bool start();
  bool loadCameraInfo();
  bool publish();
  void stop();
  bool changePipelineState(GstState state);
  void onTimer();
  void clearBufferedSample();
  bool hasSubscribers() const;

private:
  // ROS elements
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Node status booleans
  std::atomic<bool> is_started_;
  std::atomic<bool> stop_requested_;
  std::atomic<bool> quit_requested_;

  // Gstreamer elements
  GstElement* gst_pipeline_;
  GstElement* gst_sink_;

  // General gstreamer configuration
  std::string camera_config_;
  std::string camera_name_;
  std::string camera_info_;
  std::string frame_id_;
  std::string image_encoding_;
  std::string base_frame_id_;
  int retry_count_;
  int camera_type_;
  double time_offset_;
  int image_width_;
  int image_height_;
  int pixel_size_;
  bool use_gst_timestamps_;
  bool is_first_initialize_;
  bool debug_;

  // Maximum publication rate variables
  double max_pub_rate_hz_;
  std::chrono::nanoseconds timer_period_;
  rclcpp::Time last_retry_time_;
};
}  // namespace ros_kortex_vision
