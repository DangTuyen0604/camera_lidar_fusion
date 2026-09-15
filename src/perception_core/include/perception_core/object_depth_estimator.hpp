#ifndef PERCEPTION_CORE__OBJECT_DEPTH_ESTIMATOR_HPP_
#define PERCEPTION_CORE__OBJECT_DEPTH_ESTIMATOR_HPP_

#include <cstddef>
#include <vector>

#include "perception_core/types.hpp"

namespace perception_core
{

struct DepthEstimatorParameters
{
  std::size_t min_lidar_points{3};
  double bbox_shrink_ratio{0.7};
  double min_depth{1.0};
  double max_depth{80.0};
  double depth_cluster_gap{1.0};
  double outlier_threshold{2.0};
  double minimum_outlier_window{0.25};
};

class ObjectDepthEstimator
{
public:
  explicit ObjectDepthEstimator(
    DepthEstimatorParameters parameters = DepthEstimatorParameters{});

  DepthEstimate estimate(
    const BoundingBox2D & bounding_box,
    const std::vector<ProjectedPoint> & projected_points) const;

  const DepthEstimatorParameters & parameters() const noexcept;

private:
  DepthEstimatorParameters parameters_;
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__OBJECT_DEPTH_ESTIMATOR_HPP_
