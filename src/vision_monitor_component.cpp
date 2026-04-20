#include <ros_kortex_vision/vision_monitor_component.hpp>

#include <algorithm>
#include <cctype>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>

namespace
{
constexpr auto NODE_NAME = "kinova_vision_monitor_component";
const std::string DEBUG_PARAM = "debug";
const std::string REPORT_PERIOD_PARAM = "report_period_sec";
const std::string QOS_RELIABILITY_PARAM = "qos_reliability";
const std::string QOS_HISTORY_DEPTH_PARAM = "qos_history_depth";

rclcpp::ReliabilityPolicy parseReliability(const std::string& reliability)
{
  std::string value = reliability;
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (value == "reliable")
  {
    return rclcpp::ReliabilityPolicy::Reliable;
  }

  if (value == "best_effort")
  {
    return rclcpp::ReliabilityPolicy::BestEffort;
  }

  throw std::invalid_argument("Unsupported qos_reliability value: " + reliability);
}
}  // namespace

namespace ros_kortex_vision
{
VisionMonitorComponent::VisionMonitorComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node(NODE_NAME, options)
  , debug_(false)
  , qos_reliability_("reliable")
  , qos_history_depth_(10)
  , report_period_sec_(1.0)
  , frame_count_(0)
  , latency_sum_ms_(0.0)
  , latency_min_ms_(std::numeric_limits<double>::max())
  , latency_max_ms_(0.0)
  , last_report_time_(now())
  , received_frame_count_(0)
{
  declare_parameter<bool>(DEBUG_PARAM, false);
  debug_ = get_parameter(DEBUG_PARAM).as_bool();

  declare_parameter<std::string>(QOS_RELIABILITY_PARAM, qos_reliability_);
  qos_reliability_ = get_parameter(QOS_RELIABILITY_PARAM).as_string();

  declare_parameter<int>(QOS_HISTORY_DEPTH_PARAM, static_cast<int>(qos_history_depth_));
  qos_history_depth_ = static_cast<std::size_t>(std::max<int64_t>(get_parameter(QOS_HISTORY_DEPTH_PARAM).as_int(), 1));

  declare_parameter<double>(REPORT_PERIOD_PARAM, 1.0);
  report_period_sec_ = get_parameter(REPORT_PERIOD_PARAM).as_double();

  const auto reliability = parseReliability(qos_reliability_);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_history_depth_));
  qos.durability_volatile();
  qos.reliability(reliability);

  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", qos, std::bind(&VisionMonitorComponent::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Using QoS reliability='%s' depth=%zu", qos_reliability_.c_str(), qos_history_depth_);

  report_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::duration<double>(std::max(report_period_sec_, 0.1))),
                                    std::bind(&VisionMonitorComponent::reportStats, this));
}

void VisionMonitorComponent::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  const auto current_time = now();
  const double latency_ms = (current_time - msg->header.stamp).seconds() * 1000.0;

  ++received_frame_count_;
  frame_count_++;
  latency_sum_ms_ += latency_ms;
  latency_min_ms_ = std::min(latency_min_ms_, latency_ms);
  latency_max_ms_ = std::max(latency_max_ms_, latency_ms);

  if (debug_)
  {
    RCLCPP_INFO(get_logger(), "Monitor received frame=%llu stamp=%.9f pointer=%p data=%p latency=%.3f ms", static_cast<unsigned long long>(received_frame_count_),
                msg->header.stamp.seconds(), static_cast<const void*>(msg.get()), static_cast<const void*>(msg->data.data()),
                latency_ms);
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
