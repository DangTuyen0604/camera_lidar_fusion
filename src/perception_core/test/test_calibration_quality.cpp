#include <cmath>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "perception_core/calibration_quality.hpp"

namespace
{

perception_core::CalibrationData calibration()
{
  perception_core::CalibrationData result;
  result.image_width = 100;
  result.image_height = 80;
  result.projection_matrix <<
    100.0, 0.0, 50.0, 0.0,
    0.0, 100.0, 40.0, 0.0,
    0.0, 0.0, 1.0, 0.0;
  return result;
}

TEST(CalibrationQuality, AcceptsMatchingCalibrationAndObservations)
{
  const auto reference = calibration();
  const perception_core::CalibrationQualityEvaluator evaluator;
  const auto result = evaluator.evaluate(
    reference, reference,
    {{Eigen::Vector3d(0.0, 0.0, 10.0), 1.0}},
    {Eigen::Vector2d(50.0, 40.0)});

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.health, perception_core::CalibrationHealth::Ok);
  EXPECT_DOUBLE_EQ(result.alignment_score, 1.0);
  EXPECT_DOUBLE_EQ(result.projection_error_px, 0.0);
  EXPECT_EQ(result.correspondence_count, 1U);
}

TEST(CalibrationQuality, DetectsTranslationAndRotationDrift)
{
  const auto reference = calibration();
  auto candidate = reference;
  candidate.lidar_to_camera.topLeftCorner<3, 3>() =
    Eigen::AngleAxisd(2.0 * 3.14159265358979323846 / 180.0, Eigen::Vector3d::UnitZ())
    .toRotationMatrix();
  candidate.lidar_to_camera(0, 3) = 0.2;

  const perception_core::CalibrationQualityEvaluator evaluator;
  const auto result = evaluator.evaluate(reference, candidate);
  EXPECT_FALSE(result.valid);
  EXPECT_NEAR(result.translation_drift_m, 0.2, 1.0e-12);
  EXPECT_NEAR(result.rotation_drift_deg, 2.0, 1.0e-9);
  EXPECT_EQ(result.health, perception_core::CalibrationHealth::Warning);
}

TEST(CalibrationQuality, RejectsMismatchedCorrespondences)
{
  const auto value = calibration();
  const perception_core::CalibrationQualityEvaluator evaluator;
  EXPECT_THROW(
    evaluator.evaluate(value, value, {{Eigen::Vector3d::Ones(), 1.0}}, {}),
    std::invalid_argument);
}

TEST(CalibrationQuality, EscalatesLargeDriftToError)
{
  const auto reference = calibration();
  auto candidate = reference;
  candidate.lidar_to_camera(0, 3) = 0.25;
  const perception_core::CalibrationQualityEvaluator evaluator;
  const auto result = evaluator.evaluate(reference, candidate);
  EXPECT_EQ(result.health, perception_core::CalibrationHealth::Error);
  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.alignment_score, 0.0);
}

}  // namespace
