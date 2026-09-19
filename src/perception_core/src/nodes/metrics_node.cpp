#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "fusion_interfaces/msg/detection2_d_array.hpp"
#include "fusion_interfaces/msg/calibration_status.hpp"
#include "fusion_interfaces/msg/fused_detection_array.hpp"
#include "fusion_interfaces/msg/pipeline_metrics.hpp"
#include "fusion_interfaces/msg/sync_status.hpp"
#include "perception_core/latency_tracker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception_core
{

class MetricsNode : public rclcpp::Node
{
public:
  MetricsNode()
  : Node("metrics_node"), end_to_end_latency_(200), frame_intervals_(200)
  {
    const auto detections_topic = declare_parameter<std::string>(
      "detections_topic", "/detections_2d");
    const auto fused_topic = declare_parameter<std::string>(
      "fused_detections_topic", "/fusion/detections_3d");
    const auto metrics_topic = declare_parameter<std::string>(
      "metrics_topic", "/fusion/metrics");
    const auto sync_status_topic = declare_parameter<std::string>(
      "sync_status_topic", "/fusion/sync_status");
    const auto calibration_status_topic = declare_parameter<std::string>(
      "calibration_status_topic", "/fusion/calibration_status");
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
            frame_intervals_.observe(seconds * 1000.0);
          }
        }
        last_fused_time_ = current;
      });
    sync_subscription_ = create_subscription<fusion_interfaces::msg::SyncStatus>(
      sync_status_topic, 10,
      [this](fusion_interfaces::msg::SyncStatus::ConstSharedPtr message) {
        dropped_frames_ = message->dropped_messages;
      });
    calibration_subscription_ = create_subscription<fusion_interfaces::msg::CalibrationStatus>(
      calibration_status_topic, 10,
      [this](fusion_interfaces::msg::CalibrationStatus::ConstSharedPtr message) {
        calibration_healthy_ = message->valid;
        calibration_alignment_score_ = message->alignment_score;
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
    const auto latency = end_to_end_latency_.statistics();
    message.end_to_end_ms = static_cast<float>(latency.mean_ms);
    message.end_to_end_p50_ms = static_cast<float>(latency.percentile_50_ms);
    message.end_to_end_p95_ms = static_cast<float>(latency.percentile_95_ms);
    message.fusion_ms = std::max(0.0F, message.end_to_end_ms - message.inference_ms);
    const double mean_interval_ms = frame_intervals_.statistics().mean_ms;
    message.processing_hz = mean_interval_ms > 0.0 ?
      static_cast<float>(1000.0 / mean_interval_ms) : 0.0F;
    message.detection_count = detection_count_;
    message.fused_detection_count = fused_detection_count_;
    message.dropped_frames = dropped_frames_;
    message.calibration_healthy = calibration_healthy_;
    message.calibration_alignment_score = calibration_alignment_score_;
    publisher_->publish(message);
  }

  float inference_ms_{0.0F};
  std::uint32_t detection_count_{0};
  std::uint32_t fused_detection_count_{0};
  std::uint64_t dropped_frames_{0};
  bool calibration_healthy_{false};
  float calibration_alignment_score_{0.0F};
  std::chrono::steady_clock::time_point last_fused_time_{};
  LatencyTracker end_to_end_latency_;
  LatencyTracker frame_intervals_;
  rclcpp::Publisher<fusion_interfaces::msg::PipelineMetrics>::SharedPtr publisher_;
  rclcpp::Subscription<fusion_interfaces::msg::Detection2DArray>::SharedPtr
    detections_subscription_;
  rclcpp::Subscription<fusion_interfaces::msg::FusedDetectionArray>::SharedPtr
    fused_subscription_;
  rclcpp::Subscription<fusion_interfaces::msg::SyncStatus>::SharedPtr sync_subscription_;
  rclcpp::Subscription<fusion_interfaces::msg::CalibrationStatus>::SharedPtr
    calibration_subscription_;
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
