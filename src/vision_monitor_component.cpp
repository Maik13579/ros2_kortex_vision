#include <ros_kortex_vision/vision_monitor_component.hpp>

#include <algorithm>

#include <rclcpp_components/register_node_macro.hpp>

namespace
{
constexpr auto NODE_NAME = "kinova_vision_monitor_component";
const std::string DEBUG_PARAM = "debug";
const std::string REPORT_PERIOD_PARAM = "report_period_sec";
}  // namespace

namespace ros_kortex_vision
{
VisionMonitorComponent::VisionMonitorComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node(NODE_NAME, options)
  , debug_(false)
  , report_period_sec_(1.0)
  , frame_count_(0)
  , latency_sum_ms_(0.0)
  , latency_min_ms_(std::numeric_limits<double>::max())
  , latency_max_ms_(0.0)
  , last_report_time_(now())
{
  declare_parameter<bool>(DEBUG_PARAM, false);
  debug_ = get_parameter(DEBUG_PARAM).as_bool();

  declare_parameter<double>(REPORT_PERIOD_PARAM, 1.0);
  report_period_sec_ = get_parameter(REPORT_PERIOD_PARAM).as_double();

  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      std::bind(&VisionMonitorComponent::imageCallback, this, std::placeholders::_1));

  report_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::duration<double>(std::max(report_period_sec_, 0.1))),
                                    std::bind(&VisionMonitorComponent::reportStats, this));
}

void VisionMonitorComponent::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  const auto current_time = now();
  const double latency_ms = (current_time - msg->header.stamp).seconds() * 1000.0;

  frame_count_++;
  latency_sum_ms_ += latency_ms;
  latency_min_ms_ = std::min(latency_min_ms_, latency_ms);
  latency_max_ms_ = std::max(latency_max_ms_, latency_ms);

  if (debug_)
  {
    RCLCPP_INFO(get_logger(), "Monitor received image pointer=%p data=%p latency=%.3f ms", static_cast<const void*>(msg.get()),
                static_cast<const void*>(msg->data.data()), latency_ms);
  }
}

void VisionMonitorComponent::reportStats()
{
  const auto current_time = now();
  const double dt = (current_time - last_report_time_).seconds();
  const double hz = dt > 0.0 ? static_cast<double>(frame_count_) / dt : 0.0;
  const double avg_latency_ms = frame_count_ > 0 ? latency_sum_ms_ / static_cast<double>(frame_count_) : 0.0;
  const double min_latency_ms = frame_count_ > 0 ? latency_min_ms_ : 0.0;
  const double max_latency_ms = frame_count_ > 0 ? latency_max_ms_ : 0.0;

  RCLCPP_INFO(get_logger(), "image_raw: hz=%.3f avg_latency=%.3f ms min_latency=%.3f ms max_latency=%.3f ms frames=%zu",
              hz, avg_latency_ms, min_latency_ms, max_latency_ms, frame_count_);

  frame_count_ = 0;
  latency_sum_ms_ = 0.0;
  latency_min_ms_ = std::numeric_limits<double>::max();
  latency_max_ms_ = 0.0;
  last_report_time_ = current_time;
}
}  // namespace ros_kortex_vision

RCLCPP_COMPONENTS_REGISTER_NODE(ros_kortex_vision::VisionMonitorComponent)
