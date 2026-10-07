#ifndef PERCEPTION_CORE__GROUND_SEGMENTATION_HPP_
#define PERCEPTION_CORE__GROUND_SEGMENTATION_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "perception_core/types.hpp"

namespace perception_core
{

// All heights are in the frame the points are given in. Transform clouds to a
// floor-level frame (base_footprint) first and the same values hold for any
// sensor mounting height or tilt.
struct GroundSegmentationParameters
{
  double distance_threshold{0.10};  // inlier distance to the plane [m]
  int max_iterations{100};
  double max_slope_deg{15.0};       // allowed angle between plane normal and +z
  double min_ground_height{-0.30};  // plane height at the frame origin must
  double max_ground_height{0.20};   // lie in [min, max]
  double min_inlier_ratio{0.10};    // of the candidate points near that band
  std::uint32_t seed{42};           // fixed seed: identical input, identical output
};

struct GroundPlane
{
  bool valid{false};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};  // unit, normal.z() > 0
  double offset{0.0};  // normal.dot(p) + offset == 0 on the plane
  std::size_t inliers{0};

  double signedDistance(const Eigen::Vector3d & point) const
  {
    return normal.dot(point) + offset;
  }
};

// RANSAC plane fit restricted to points near the expected ground height, so
// tall obstacles never seed a hypothesis, followed by a least-squares refit
// on the inliers. Without a plane that satisfies the slope/height limits every
// point is an obstacle: a planar 2D scan above the floor is never "ground".
class RansacGroundSegmenter
{
public:
  explicit RansacGroundSegmenter(
    GroundSegmentationParameters parameters = GroundSegmentationParameters{});

  GroundPlane segment(
    const std::vector<PointXYZI> & points,
    std::vector<PointXYZI> & ground,
    std::vector<PointXYZI> & obstacles);

  const GroundSegmentationParameters & parameters() const noexcept;

private:
  bool acceptable(const GroundPlane & plane) const;

  GroundSegmentationParameters parameters_;
  double min_normal_z_;
  std::vector<std::size_t> candidates_;
  std::vector<std::size_t> inliers_;
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__GROUND_SEGMENTATION_HPP_
