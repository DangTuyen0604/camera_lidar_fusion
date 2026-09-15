#include <vector>

#include <gtest/gtest.h>

#include "perception_core/object_depth_estimator.hpp"

namespace
{

perception_core::ProjectedPoint point(double u, double v, double x, double y, double depth)
{
  return {u, v, Eigen::Vector3d(x, y, depth), depth, 1.0};
}

const perception_core::BoundingBox2D kBox{0.0, 0.0, 100.0, 100.0};

TEST(ObjectDepthEstimator, EmptyBoundingBoxIsInvalid)
{
  const perception_core::ObjectDepthEstimator estimator;
  const auto result = estimator.estimate(kBox, {});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.lidar_point_count, 0U);
}

TEST(ObjectDepthEstimator, FewerThanMinimumPointsIsInvalid)
{
  const perception_core::ObjectDepthEstimator estimator;
  const auto result = estimator.estimate(kBox, {
      point(50.0, 50.0, 0.0, 0.0, 10.0),
      point(51.0, 50.0, 0.1, 0.0, 10.1)
    });
  EXPECT_FALSE(result.valid);
}

TEST(ObjectDepthEstimator, ReturnsMedianForSingleCluster)
{
  const perception_core::ObjectDepthEstimator estimator;
  const auto result = estimator.estimate(kBox, {
      point(45.0, 50.0, -0.2, 0.0, 9.8),
      point(50.0, 50.0, 0.0, 0.1, 10.0),
      point(55.0, 50.0, 0.2, 0.2, 10.2)
    });
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.depth, 10.0, 1.0e-12);
  EXPECT_TRUE(result.position.isApprox(Eigen::Vector3d(0.0, 0.1, 10.0), 1.0e-12));
  EXPECT_EQ(result.lidar_point_count, 3U);
}

TEST(ObjectDepthEstimator, RemovesDepthOutlierWithMad)
{
  const perception_core::ObjectDepthEstimator estimator;
  const auto result = estimator.estimate(kBox, {
      point(45.0, 50.0, 0.0, 0.0, 10.0),
      point(48.0, 50.0, 0.1, 0.0, 10.1),
      point(52.0, 50.0, 0.2, 0.0, 10.2),
      point(55.0, 50.0, 5.0, 0.0, 10.9)
    });
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.depth, 10.1, 1.0e-12);
  EXPECT_EQ(result.lidar_point_count, 3U);
}

TEST(ObjectDepthEstimator, ChoosesNearestValidForegroundCluster)
{
  const perception_core::ObjectDepthEstimator estimator;
  const auto result = estimator.estimate(kBox, {
      point(45.0, 50.0, 0.0, 0.0, 10.0),
      point(48.0, 50.0, 0.1, 0.0, 10.1),
      point(52.0, 50.0, 0.2, 0.0, 10.2),
      point(45.0, 55.0, 0.0, 1.0, 30.0),
      point(48.0, 55.0, 0.1, 1.0, 30.1),
      point(50.0, 55.0, 0.2, 1.0, 30.2),
      point(52.0, 55.0, 0.3, 1.0, 30.3),
      point(55.0, 55.0, 0.4, 1.0, 30.4)
    });
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.depth, 10.1, 1.0e-12);
  EXPECT_EQ(result.lidar_point_count, 3U);
}

TEST(ObjectDepthEstimator, UsesCentralShrunkBoundingBox)
{
  const perception_core::ObjectDepthEstimator estimator;
  const auto result = estimator.estimate(kBox, {
      point(5.0, 5.0, 0.0, 0.0, 5.0),
      point(6.0, 6.0, 0.0, 0.0, 5.1),
      point(7.0, 7.0, 0.0, 0.0, 5.2),
      point(45.0, 50.0, 0.0, 0.0, 12.0),
      point(50.0, 50.0, 0.1, 0.0, 12.1),
      point(55.0, 50.0, 0.2, 0.0, 12.2)
    });
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.depth, 12.1, 1.0e-12);
}

TEST(ObjectDepthEstimator, RejectsInvalidParameters)
{
  perception_core::DepthEstimatorParameters parameters;
  parameters.bbox_shrink_ratio = 1.5;
  EXPECT_THROW(
    perception_core::ObjectDepthEstimator estimator(parameters),
    std::invalid_argument);
}

}  // namespace
