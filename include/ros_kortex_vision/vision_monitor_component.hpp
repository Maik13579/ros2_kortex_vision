#pragma once

#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ros_kortex_vision
{
class VisionMonitorComponent : public rclcpp::Node
{
public:
  explicit VisionMonitorComponent(const rclcpp::NodeOptions& options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void reportStats();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  bool debug_;
  double report_period_sec_;
  std::size_t frame_count_;
  double latency_sum_ms_;
  double latency_min_ms_;
  double latency_max_ms_;
  rclcpp::Time last_report_time_;
};
}  // namespace ros_kortex_vision
