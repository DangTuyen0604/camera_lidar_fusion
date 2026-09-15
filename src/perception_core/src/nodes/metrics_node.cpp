#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "fusion_interfaces/msg/detection2_d_array.hpp"
#include "fusion_interfaces/msg/fused_detection_array.hpp"
#include "fusion_interfaces/msg/pipeline_metrics.hpp"
#include "perception_core/latency_tracker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception_core
{

class MetricsNode : public rclcpp::Node
{
public:
  MetricsNode()
  : Node("metrics_node"), end_to_end_latency_(200)
  {
    const auto detections_topic = declare_parameter<std::string>(
      "detections_topic", "/detections_2d");
    const auto fused_topic = declare_parameter<std::string>(
      "fused_detections_topic", "/fusion/detections_3d");
    const auto metrics_topic = declare_parameter<std::string>(
      "metrics_topic", "/fusion/metrics");
    const double publish_period_sec = declare_parameter<double>("publish_period_sec", 1.0);
    if (publish_period_sec <= 0.0) {
      throw std::invalid_argument("publish_period_sec must be positive");
    }

    publisher_ = create_publisher<fusion_interfaces::msg::PipelineMetrics>(metrics_topic, 10);
    detections_subscription_ = create_subscription<fusion_interfaces::msg::Detection2DArray>(
      detections_topic, 10,
      [this](fusion_interfaces::msg::Detection2DArray::ConstSharedPtr message) {
        inference_ms_ = message->inference_ms;
        detection_count_ = static_cast<std::uint32_t>(message->detections.size());
      });
    fused_subscription_ = create_subscription<fusion_interfaces::msg::FusedDetectionArray>(
      fused_topic, 10,
      [this](fusion_interfaces::msg::FusedDetectionArray::ConstSharedPtr message) {
        fused_detection_count_ = static_cast<std::uint32_t>(std::count_if(
            message->detections.begin(), message->detections.end(),
          [](const auto & detection) {return detection.valid;}));
        const rclcpp::Time stamp(message->header.stamp);
        const double latency_ms = std::max(0.0, (now() - stamp).seconds() * 1000.0);
        if (std::isfinite(latency_ms)) {
          end_to_end_latency_.observe(latency_ms);
        }
        const auto current = std::chrono::steady_clock::now();
        if (last_fused_time_.time_since_epoch().count() != 0) {
          const double seconds = std::chrono::duration<double>(current - last_fused_time_).count();
          if (seconds > 0.0) {
            processing_hz_ = static_cast<float>(1.0 / seconds);
          }
        }
        last_fused_time_ = current;
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_sec),
      std::bind(&MetricsNode::publish, this));
  }

private:
  void publish()
  {
    fusion_interfaces::msg::PipelineMetrics message;
    message.header.stamp = now();
    message.header.frame_id = "camera_optical_frame";
    message.inference_ms = inference_ms_;
    message.end_to_end_ms = static_cast<float>(end_to_end_latency_.statistics().mean_ms);
    message.fusion_ms = std::max(0.0F, message.end_to_end_ms - message.inference_ms);
    message.processing_hz = processing_hz_;
    message.detection_count = detection_count_;
    message.fused_detection_count = fused_detection_count_;
    publisher_->publish(message);
  }

  float inference_ms_{0.0F};
  float processing_hz_{0.0F};
  std::uint32_t detection_count_{0};
  std::uint32_t fused_detection_count_{0};
  std::chrono::steady_clock::time_point last_fused_time_{};
  LatencyTracker end_to_end_latency_;
  rclcpp::Publisher<fusion_interfaces::msg::PipelineMetrics>::SharedPtr publisher_;
  rclcpp::Subscription<fusion_interfaces::msg::Detection2DArray>::SharedPtr
    detections_subscription_;
  rclcpp::Subscription<fusion_interfaces::msg::FusedDetectionArray>::SharedPtr
    fused_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace perception_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_core::MetricsNode>());
  rclcpp::shutdown();
  return 0;
}
