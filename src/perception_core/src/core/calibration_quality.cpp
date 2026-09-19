#include "perception_core/calibration_quality.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Geometry>

namespace perception_core
{
namespace
{

constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

Eigen::Vector2d projectPoint(
  const CalibrationData & calibration,
  const PointXYZI & point)
{
  const Eigen::Vector4d homogeneous(
    point.position.x(), point.position.y(), point.position.z(), 1.0);
  const Eigen::Vector4d camera =
    calibration.rectification_matrix * calibration.lidar_to_camera * homogeneous;
  const Eigen::Vector3d pixel = calibration.projection_matrix * camera;
  if (!pixel.allFinite() || pixel.z() <= 1.0e-9) {
    return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  return pixel.head<2>() / pixel.z();
}

}  // namespace

CalibrationQualityEvaluator::CalibrationQualityEvaluator(
  CalibrationQualityThresholds thresholds)
: thresholds_(thresholds)
{
  if (!std::isfinite(thresholds_.maximum_projection_error_px) ||
    !std::isfinite(thresholds_.maximum_translation_drift_m) ||
    !std::isfinite(thresholds_.maximum_rotation_drift_deg) ||
    !std::isfinite(thresholds_.error_multiplier) ||
    thresholds_.maximum_projection_error_px < 0.0 ||
    thresholds_.maximum_translation_drift_m < 0.0 ||
    thresholds_.maximum_rotation_drift_deg < 0.0 || thresholds_.error_multiplier <= 1.0)
  {
    throw std::invalid_argument("Invalid calibration quality thresholds");
  }
}

CalibrationQualityResult CalibrationQualityEvaluator::evaluate(
  const CalibrationData & reference,
  const CalibrationData & candidate,
  const std::vector<PointXYZI> & lidar_points,
  const std::vector<Eigen::Vector2d> & observed_pixels) const
{
  if (lidar_points.size() != observed_pixels.size()) {
    throw std::invalid_argument("LiDAR point and observed pixel counts differ");
  }
  if (!reference.lidar_to_camera.allFinite() || !candidate.lidar_to_camera.allFinite()) {
    throw std::invalid_argument("Calibration transforms contain non-finite values");
  }

  CalibrationQualityResult result;
  result.translation_drift_m =
    (candidate.lidar_to_camera.topRightCorner<3, 1>() -
    reference.lidar_to_camera.topRightCorner<3, 1>()).norm();
  const Eigen::Matrix3d relative_rotation =
    reference.lidar_to_camera.topLeftCorner<3, 3>().transpose() *
    candidate.lidar_to_camera.topLeftCorner<3, 3>();
  const double cosine = std::clamp((relative_rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
  result.rotation_drift_deg = std::acos(cosine) * kRadiansToDegrees;

  double error_sum = 0.0;
  for (std::size_t index = 0; index < lidar_points.size(); ++index) {
    if (!observed_pixels[index].allFinite()) {
      continue;
    }
    const Eigen::Vector2d projected = projectPoint(candidate, lidar_points[index]);
    if (!projected.allFinite()) {
      continue;
    }
    error_sum += (projected - observed_pixels[index]).norm();
    ++result.correspondence_count;
  }
  if (result.correspondence_count > 0) {
    result.projection_error_px = error_sum /
      static_cast<double>(result.correspondence_count);
  }

  const auto ratio = [](double value, double threshold) {
      if (threshold == 0.0) {
        return value == 0.0 ? 0.0 : std::numeric_limits<double>::infinity();
      }
      return value / threshold;
    };
  double worst_ratio = std::max(
    ratio(result.translation_drift_m, thresholds_.maximum_translation_drift_m),
    ratio(result.rotation_drift_deg, thresholds_.maximum_rotation_drift_deg));
  if (!lidar_points.empty()) {
    worst_ratio = std::max(
      worst_ratio,
      result.correspondence_count == 0 ? std::numeric_limits<double>::infinity() :
      ratio(result.projection_error_px, thresholds_.maximum_projection_error_px));
  }
  result.alignment_score = std::isfinite(worst_ratio) ?
    std::clamp(1.0 - worst_ratio / thresholds_.error_multiplier, 0.0, 1.0) : 0.0;
  result.health = worst_ratio <= 1.0 ? CalibrationHealth::Ok :
    (worst_ratio <= thresholds_.error_multiplier ? CalibrationHealth::Warning :
    CalibrationHealth::Error);
  result.valid = result.health == CalibrationHealth::Ok;
  return result;
}

}  // namespace perception_core
