#ifndef PERCEPTION_CORE__CALIBRATION_QUALITY_HPP_
#define PERCEPTION_CORE__CALIBRATION_QUALITY_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "perception_core/types.hpp"

namespace perception_core
{

struct CalibrationQualityThresholds
{
  double maximum_projection_error_px{3.0};
  double maximum_translation_drift_m{0.10};
  double maximum_rotation_drift_deg{1.0};
};

struct CalibrationQualityResult
{
  bool valid{false};
  double projection_error_px{0.0};
  double translation_drift_m{0.0};
  double rotation_drift_deg{0.0};
  std::size_t correspondence_count{0};
};

class CalibrationQualityEvaluator
{
public:
  explicit CalibrationQualityEvaluator(
    CalibrationQualityThresholds thresholds = CalibrationQualityThresholds{});

  CalibrationQualityResult evaluate(
    const CalibrationData & reference,
    const CalibrationData & candidate,
    const std::vector<PointXYZI> & lidar_points = {},
    const std::vector<Eigen::Vector2d> & observed_pixels = {}) const;

private:
  CalibrationQualityThresholds thresholds_;
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__CALIBRATION_QUALITY_HPP_
