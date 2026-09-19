#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "fusion_interfaces/msg/calibration_status.hpp"
#include "perception_core/calibration_loader.hpp"
#include "perception_core/calibration_quality.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception_core
{

class CalibrationMonitorNode : public rclcpp::Node
{
public:
  CalibrationMonitorNode()
  : Node("calibration_monitor_node")
  {
    intrinsics_path_ = declare_parameter<std::string>("intrinsics_path", "");
    reference_extrinsics_path_ = declare_parameter<std::string>(
      "reference_extrinsics_path", "");
    candidate_extrinsics_path_ = declare_parameter<std::string>(
      "candidate_extrinsics_path", reference_extrinsics_path_);
    const double period_sec = declare_parameter<double>("period_sec", 1.0);
    CalibrationQualityThresholds thresholds;
    thresholds.maximum_projection_error_px = declare_parameter<double>(
      "maximum_projection_error_px", 3.0);
    thresholds.maximum_translation_drift_m = declare_parameter<double>(
      "maximum_translation_drift_m", 0.10);
    thresholds.maximum_rotation_drift_deg = declare_parameter<double>(
      "maximum_rotation_drift_deg", 1.0);
    thresholds.error_multiplier = declare_parameter<double>("error_multiplier", 2.0);
    recovery_samples_ = declare_parameter<int>("recovery_samples", 2);
    evaluator_ = std::make_unique<CalibrationQualityEvaluator>(thresholds);

    if (period_sec <= 0.0 || recovery_samples_ <= 0) {
      throw std::invalid_argument("period_sec and recovery_samples must be positive");
    }
    publisher_ = create_publisher<fusion_interfaces::msg::CalibrationStatus>(
      "/fusion/calibration_status", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_sec),
      std::bind(&CalibrationMonitorNode::evaluate, this));
  }

private:
  void evaluate()
  {
    fusion_interfaces::msg::CalibrationStatus status;
    status.header.stamp = now();
    status.header.frame_id = "camera_optical_frame";
    try {
      if (intrinsics_path_.empty() || reference_extrinsics_path_.empty() ||
        candidate_extrinsics_path_.empty())
      {
        throw std::runtime_error("Calibration file paths are not configured");
      }
      const auto reference = CalibrationLoader::loadFromYaml(
        intrinsics_path_, reference_extrinsics_path_);
      const auto candidate = CalibrationLoader::loadFromYaml(
        intrinsics_path_, candidate_extrinsics_path_);
      const auto result = evaluator_->evaluate(reference, candidate);
      auto health = result.health;
      if (health == CalibrationHealth::Ok) {
        ++consecutive_healthy_;
        if (last_health_ != CalibrationHealth::Ok && consecutive_healthy_ < recovery_samples_) {
          health = CalibrationHealth::Warning;
        }
      } else {
        consecutive_healthy_ = 0;
      }
      last_health_ = health;
      status.state = static_cast<std::uint8_t>(health);
      status.valid = health == CalibrationHealth::Ok;
      status.alignment_score = static_cast<float>(result.alignment_score);
      status.projection_error_px = static_cast<float>(result.projection_error_px);
      status.translation_drift_m = static_cast<float>(result.translation_drift_m);
      status.rotation_drift_deg = static_cast<float>(result.rotation_drift_deg);
      status.message = health == CalibrationHealth::Ok ? "calibration within thresholds" :
        (health == CalibrationHealth::Warning ? "calibration warning or recovering" :
        "calibration drift exceeds error threshold");
    } catch (const std::exception & error) {
      consecutive_healthy_ = 0;
      last_health_ = CalibrationHealth::Error;
      status.state = fusion_interfaces::msg::CalibrationStatus::STATE_ERROR;
      status.valid = false;
      status.message = error.what();
    }
    publisher_->publish(status);
  }

  std::string intrinsics_path_;
  std::string reference_extrinsics_path_;
  std::string candidate_extrinsics_path_;
  std::unique_ptr<CalibrationQualityEvaluator> evaluator_;
  int recovery_samples_{2};
  int consecutive_healthy_{0};
  CalibrationHealth last_health_{CalibrationHealth::Ok};
  rclcpp::Publisher<fusion_interfaces::msg::CalibrationStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace perception_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_core::CalibrationMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
