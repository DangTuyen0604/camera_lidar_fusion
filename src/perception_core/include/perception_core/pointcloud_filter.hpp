#ifndef PERCEPTION_CORE__POINTCLOUD_FILTER_HPP_
#define PERCEPTION_CORE__POINTCLOUD_FILTER_HPP_

#include <vector>

#include "perception_core/types.hpp"

namespace perception_core
{

struct PointCloudFilterParameters
{
  Eigen::Vector3d minimum{-50.0, -30.0, -3.0};
  Eigen::Vector3d maximum{100.0, 30.0, 5.0};
  double minimum_intensity{0.0};
  double voxel_size{0.0};
  // Drop points closer than this to the frame's z axis (horizontal range).
  // In a robot frame this is the near-range/self filter of a 2D scan's
  // range_min: returns from the robot body or from a station it is docked
  // against, which the costmaps would otherwise put inside the footprint.
  double minimum_radius{0.0};
};

class PointCloudFilter
{
public:
  explicit PointCloudFilter(
    PointCloudFilterParameters parameters = PointCloudFilterParameters{});

  std::vector<PointXYZI> filter(const std::vector<PointXYZI> & points) const;
  // Same as above, writing into `output` and reusing its capacity.
  void filter(const std::vector<PointXYZI> & points, std::vector<PointXYZI> & output) const;
  const PointCloudFilterParameters & parameters() const noexcept;

private:
  PointCloudFilterParameters parameters_;
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__POINTCLOUD_FILTER_HPP_
