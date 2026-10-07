#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "perception_core/euclidean_clustering.hpp"
#include "perception_core/ground_segmentation.hpp"
#include "perception_core/pointcloud_preprocessor.hpp"

namespace
{

using perception_core::PointXYZI;

constexpr double kPi = 3.14159265358979323846;

// Floor z = slope_x * x + height on a 10 m x 10 m patch, with +-noise.
std::vector<PointXYZI> floorPatch(double height, double slope_x, double noise, int seed = 1)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> jitter(-noise, noise);
  std::vector<PointXYZI> points;
  for (double x = -5.0; x <= 5.0; x += 0.2) {
    for (double y = -5.0; y <= 5.0; y += 0.2) {
      points.push_back({Eigen::Vector3d(x, y, slope_x * x + height + jitter(rng)), 1.0});
    }
  }
  return points;
}

void appendBox(
  std::vector<PointXYZI> & points, const Eigen::Vector3d & low,
  const Eigen::Vector3d & high, double step = 0.1)
{
  for (double x = low.x(); x <= high.x() + 1e-9; x += step) {
    for (double y = low.y(); y <= high.y() + 1e-9; y += step) {
      for (double z = low.z(); z <= high.z() + 1e-9; z += step) {
        points.push_back({Eigen::Vector3d(x, y, z), 1.0});
      }
    }
  }
}

}  // namespace

TEST(RansacGroundSegmenter, SeparatesFloorFromBoxes)
{
  auto points = floorPatch(0.0, 0.0, 0.02);
  const std::size_t floor_count = points.size();
  appendBox(points, {1.0, 1.0, 0.3}, {1.5, 1.5, 1.0});
  appendBox(points, {-3.0, 2.0, 0.3}, {-2.5, 2.5, 0.6});
  const std::size_t box_count = points.size() - floor_count;

  perception_core::RansacGroundSegmenter segmenter;
  std::vector<PointXYZI> ground;
  std::vector<PointXYZI> obstacles;
  const auto plane = segmenter.segment(points, ground, obstacles);

  ASSERT_TRUE(plane.valid);
  EXPECT_NEAR(plane.normal.z(), 1.0, 1e-3);
  EXPECT_NEAR(-plane.offset / plane.normal.z(), 0.0, 0.01);
  EXPECT_EQ(ground.size(), floor_count);
  EXPECT_EQ(obstacles.size(), box_count);
}

TEST(RansacGroundSegmenter, FitsGentleSlopeAndSensorHeightBand)
{
  // 5 degree ramp, expressed in a frame 1.73 m above it (KITTI's Velodyne).
  auto points = floorPatch(-1.73, std::tan(5.0 * kPi / 180.0), 0.02);
  perception_core::GroundSegmentationParameters parameters;
  parameters.min_ground_height = -2.0;
  parameters.max_ground_height = -1.5;
  perception_core::RansacGroundSegmenter segmenter(parameters);
  std::vector<PointXYZI> ground;
  std::vector<PointXYZI> obstacles;
  const auto plane = segmenter.segment(points, ground, obstacles);

  ASSERT_TRUE(plane.valid);
  EXPECT_NEAR(std::acos(plane.normal.z()) * 180.0 / kPi, 5.0, 0.5);
  EXPECT_EQ(obstacles.size(), 0U);
}

TEST(RansacGroundSegmenter, RejectsPlanesOutsideTheGroundBand)
{
  // A planar 2D scan 0.35 m above the floor is coplanar but is not ground.
  std::vector<PointXYZI> scan;
  for (int beam = 0; beam < 360; ++beam) {
    const double angle = beam * kPi / 180.0;
    scan.push_back({Eigen::Vector3d(3.0 * std::cos(angle), 3.0 * std::sin(angle), 0.35), 1.0});
  }
  perception_core::RansacGroundSegmenter segmenter;
  std::vector<PointXYZI> ground;
  std::vector<PointXYZI> obstacles;
  EXPECT_FALSE(segmenter.segment(scan, ground, obstacles).valid);
  EXPECT_TRUE(ground.empty());
  EXPECT_EQ(obstacles.size(), scan.size());
}

TEST(RansacGroundSegmenter, RejectsSlopesSteeperThanAllowed)
{
  auto ramp = floorPatch(0.0, std::tan(30.0 * kPi / 180.0), 0.0);
  perception_core::RansacGroundSegmenter segmenter;  // 15 degree limit
  std::vector<PointXYZI> ground;
  std::vector<PointXYZI> obstacles;
  EXPECT_FALSE(segmenter.segment(ramp, ground, obstacles).valid);
}

TEST(RansacGroundSegmenter, IsDeterministicAndHandlesTinyInput)
{
  auto points = floorPatch(0.0, 0.0, 0.05, 7);
  appendBox(points, {0.0, 0.0, 0.2}, {0.4, 0.4, 0.8});
  perception_core::RansacGroundSegmenter segmenter;
  std::vector<PointXYZI> ground_a, obstacles_a, ground_b, obstacles_b;
  const auto a = segmenter.segment(points, ground_a, obstacles_a);
  const auto b = segmenter.segment(points, ground_b, obstacles_b);
  EXPECT_TRUE(a.normal.isApprox(b.normal));
  EXPECT_EQ(ground_a.size(), ground_b.size());

  std::vector<PointXYZI> two{{Eigen::Vector3d::Zero(), 0.0}, {Eigen::Vector3d::UnitX(), 0.0}};
  EXPECT_FALSE(segmenter.segment(two, ground_a, obstacles_a).valid);
  EXPECT_EQ(obstacles_a.size(), 2U);
  EXPECT_FALSE(segmenter.segment({}, ground_a, obstacles_a).valid);
}

TEST(RansacGroundSegmenter, RejectsInvalidParameters)
{
  perception_core::GroundSegmentationParameters parameters;
  parameters.distance_threshold = 0.0;
  EXPECT_THROW(perception_core::RansacGroundSegmenter{parameters}, std::invalid_argument);
  parameters = {};
  parameters.max_slope_deg = 90.0;
  EXPECT_THROW(perception_core::RansacGroundSegmenter{parameters}, std::invalid_argument);
}

TEST(EuclideanClusterer, SplitsSeparatedObjects)
{
  std::vector<PointXYZI> points;
  appendBox(points, {0.0, 0.0, 0.0}, {0.4, 0.4, 0.4});
  appendBox(points, {2.0, 0.0, 0.0}, {2.4, 0.4, 0.4});
  appendBox(points, {-3.0, -3.0, 0.0}, {-2.8, -2.8, 0.2});

  perception_core::EuclideanClusterer clusterer({0.3, 5, 1000});
  const auto & clusters = clusterer.cluster(points);
  ASSERT_EQ(clusters.size(), 3U);
  EXPECT_TRUE(clusters[0].centroid.isApprox(Eigen::Vector3d(0.2, 0.2, 0.2), 1e-9));
  EXPECT_TRUE(clusters[1].minimum.isApprox(Eigen::Vector3d(2.0, 0.0, 0.0), 1e-9));
  EXPECT_TRUE(clusters[1].maximum.isApprox(Eigen::Vector3d(2.4, 0.4, 0.4), 1e-9));
  std::size_t total = 0;
  for (const auto & cluster : clusters) {
    total += cluster.indices.size();
  }
  EXPECT_EQ(total, points.size());
}

TEST(EuclideanClusterer, ChainsPointsAcrossCellBoundariesAndNegativeCoordinates)
{
  // A line with 0.25 m spacing crosses many 0.3 m cells and the origin.
  std::vector<PointXYZI> line;
  for (double x = -2.0; x <= 2.0 + 1e-9; x += 0.25) {
    line.push_back({Eigen::Vector3d(x, -0.01, 0.0), 1.0});
  }
  perception_core::EuclideanClusterer clusterer({0.3, 1, 1000});
  ASSERT_EQ(clusterer.cluster(line).size(), 1U);

  // A gap wider than the tolerance splits it.
  line.push_back({Eigen::Vector3d(2.6, -0.01, 0.0), 1.0});
  EXPECT_EQ(clusterer.cluster(line).size(), 2U);
}

TEST(EuclideanClusterer, AppliesSizeLimits)
{
  std::vector<PointXYZI> points;
  appendBox(points, {0.0, 0.0, 0.0}, {0.2, 0.2, 0.2});      // 27 points
  points.push_back({Eigen::Vector3d(5.0, 5.0, 5.0), 1.0});  // isolated noise
  perception_core::EuclideanClusterer small({0.15, 2, 1000});
  EXPECT_EQ(small.cluster(points).size(), 1U);
  perception_core::EuclideanClusterer capped({0.15, 1, 10});
  ASSERT_EQ(capped.cluster(points).size(), 1U);
  EXPECT_EQ(capped.cluster(points)[0].indices.size(), 1U);
  EXPECT_TRUE(small.cluster({}).empty());
  EXPECT_THROW(perception_core::EuclideanClusterer({0.0, 1, 10}), std::invalid_argument);
}

TEST(PointCloudPreprocessor, RunsAllStagesOnASyntheticScene)
{
  auto points = floorPatch(0.0, 0.0, 0.02);
  appendBox(points, {1.0, 1.0, 0.3}, {1.5, 1.5, 1.0});
  appendBox(points, {-3.0, 2.0, 0.3}, {-2.5, 2.5, 0.6});
  appendBox(points, {20.0, 0.0, 0.3}, {20.5, 0.5, 0.6});  // cropped away

  perception_core::PreprocessorParameters parameters;
  parameters.filter.minimum = Eigen::Vector3d(-10.0, -10.0, -1.0);
  parameters.filter.maximum = Eigen::Vector3d(10.0, 10.0, 3.0);
  parameters.clustering = {0.3, 5, 100000};
  perception_core::PointCloudPreprocessor preprocessor(parameters);
  const auto output = preprocessor.process(points);

  EXPECT_TRUE(output.plane.valid);
  EXPECT_EQ(output.clusters->size(), 2U);
  EXPECT_EQ(output.filtered->size(), output.ground->size() + output.obstacles->size());
  EXPECT_GE(output.timing.total_ms, output.timing.cluster_ms);

  parameters.remove_ground = false;
  parameters.cluster = false;
  perception_core::PointCloudPreprocessor passthrough(parameters);
  const auto raw = passthrough.process(points);
  EXPECT_TRUE(raw.ground->empty());
  EXPECT_EQ(raw.obstacles->size(), raw.filtered->size());
  EXPECT_TRUE(raw.clusters->empty());
}
