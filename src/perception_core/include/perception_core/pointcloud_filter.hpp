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
};

class PointCloudFilter
{
public:
  explicit PointCloudFilter(
    PointCloudFilterParameters parameters = PointCloudFilterParameters{});

  std::vector<PointXYZI> filter(const std::vector<PointXYZI> & points) const;
  const PointCloudFilterParameters & parameters() const noexcept;

private:
  PointCloudFilterParameters parameters_;
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__POINTCLOUD_FILTER_HPP_
