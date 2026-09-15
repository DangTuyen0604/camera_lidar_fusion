#include <limits>

#include <gtest/gtest.h>

#include "perception_core/pointcloud_filter.hpp"

TEST(PointCloudFilter, AppliesBoundsIntensityAndFiniteChecks)
{
  perception_core::PointCloudFilterParameters parameters;
  parameters.minimum = Eigen::Vector3d(-1.0, -1.0, -1.0);
  parameters.maximum = Eigen::Vector3d(1.0, 1.0, 1.0);
  parameters.minimum_intensity = 0.25;
  const perception_core::PointCloudFilter filter(parameters);

  const auto result = filter.filter({
    {Eigen::Vector3d(0.0, 0.0, 0.0), 0.5},
    {Eigen::Vector3d(2.0, 0.0, 0.0), 0.5},
    {Eigen::Vector3d(0.0, 0.0, 0.0), 0.1},
    {Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0), 1.0}
    });

  ASSERT_EQ(result.size(), 1U);
  EXPECT_DOUBLE_EQ(result.front().intensity, 0.5);
}

TEST(PointCloudFilter, KeepsHighestIntensityPointPerVoxel)
{
  perception_core::PointCloudFilterParameters parameters;
  parameters.minimum = Eigen::Vector3d::Constant(-10.0);
  parameters.maximum = Eigen::Vector3d::Constant(10.0);
  parameters.voxel_size = 1.0;
  const perception_core::PointCloudFilter filter(parameters);

  const auto result = filter.filter({
    {Eigen::Vector3d(0.1, 0.1, 0.1), 0.2},
    {Eigen::Vector3d(0.8, 0.8, 0.8), 0.9},
    {Eigen::Vector3d(1.1, 0.1, 0.1), 0.4}
    });

  ASSERT_EQ(result.size(), 2U);
  EXPECT_DOUBLE_EQ(result[0].intensity, 0.9);
  EXPECT_DOUBLE_EQ(result[1].intensity, 0.4);
}

TEST(PointCloudFilter, RejectsInvalidBounds)
{
  perception_core::PointCloudFilterParameters parameters;
  parameters.maximum.x() = parameters.minimum.x();
  EXPECT_THROW(perception_core::PointCloudFilter filter(parameters), std::invalid_argument);
}
