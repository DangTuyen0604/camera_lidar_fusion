#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "perception_core/lidar_projector.hpp"

namespace
{

perception_core::CalibrationData makeCalibration()
{
  perception_core::CalibrationData calibration;
  calibration.image_width = 100;
  calibration.image_height = 80;
  calibration.projection_matrix <<
    100.0, 0.0, 50.0, 0.0,
    0.0, 100.0, 40.0, 0.0,
    0.0, 0.0, 1.0, 0.0;
  return calibration;
}

TEST(LidarProjector, ProjectsPointAndPreservesCameraCoordinatesAndIntensity)
{
  const perception_core::LidarProjector projector(makeCalibration());
  const auto output = projector.project({
      {Eigen::Vector3d(0.0, 0.0, 10.0), 0.75}
    });

  ASSERT_EQ(output.size(), 1U);
  EXPECT_DOUBLE_EQ(output[0].u, 50.0);
  EXPECT_DOUBLE_EQ(output[0].v, 40.0);
  EXPECT_DOUBLE_EQ(output[0].depth, 10.0);
  EXPECT_DOUBLE_EQ(output[0].intensity, 0.75);
  EXPECT_TRUE(output[0].camera_point.isApprox(Eigen::Vector3d(0.0, 0.0, 10.0)));
}

TEST(LidarProjector, FiltersBehindOutsideTooFarAndNonFinitePoints)
{
  const perception_core::LidarProjector projector(makeCalibration());
  const auto output = projector.project({
      {Eigen::Vector3d(0.0, 0.0, -2.0), 1.0},
      {Eigen::Vector3d(20.0, 0.0, 10.0), 1.0},
      {Eigen::Vector3d(0.0, 0.0, 90.0), 1.0},
      {Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 10.0), 1.0},
      {Eigen::Vector3d(0.0, 0.0, 20.0), 0.5}
    }, 100, 80, 0.1, 80.0);

  ASSERT_EQ(output.size(), 1U);
  EXPECT_DOUBLE_EQ(output[0].depth, 20.0);
}

TEST(LidarProjector, HandlesEmptyCloud)
{
  const perception_core::LidarProjector projector(makeCalibration());
  EXPECT_TRUE(projector.project({}).empty());
}

TEST(LidarProjector, RejectsInvalidDepthRange)
{
  const perception_core::LidarProjector projector(makeCalibration());
  EXPECT_THROW(projector.project({}, 100, 80, 2.0, 1.0), std::invalid_argument);
}

}  // namespace
