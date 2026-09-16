#include "perception_core/object_depth_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace perception_core
{
namespace
{

double median(std::vector<double> values)
{
  if (values.empty()) {
    throw std::invalid_argument("Cannot calculate median of an empty vector");
  }
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2 != 0) {
    return upper;
  }
  std::nth_element(values.begin(), values.begin() + middle - 1, values.begin() + middle);
  return 0.5 * (values[middle - 1] + upper);
}

Eigen::Vector3d medianPosition(const std::vector<const ProjectedPoint *> & points)
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  x.reserve(points.size());
  y.reserve(points.size());
  z.reserve(points.size());
  for (const ProjectedPoint * point : points) {
    x.push_back(point->camera_point.x());
    y.push_back(point->camera_point.y());
    z.push_back(point->camera_point.z());
  }
  return Eigen::Vector3d(median(std::move(x)), median(std::move(y)), median(std::move(z)));
}

}  // namespace

ObjectDepthEstimator::ObjectDepthEstimator(DepthEstimatorParameters parameters)
: parameters_(parameters)
{
  if (parameters_.min_lidar_points == 0 ||
    !std::isfinite(parameters_.bbox_shrink_ratio) ||
    parameters_.bbox_shrink_ratio <= 0.0 || parameters_.bbox_shrink_ratio > 1.0 ||
    !std::isfinite(parameters_.min_depth) || parameters_.min_depth < 0.0 ||
    !std::isfinite(parameters_.max_depth) || parameters_.max_depth <= parameters_.min_depth ||
    !std::isfinite(parameters_.depth_cluster_gap) || parameters_.depth_cluster_gap <= 0.0 ||
    !std::isfinite(parameters_.outlier_threshold) || parameters_.outlier_threshold <= 0.0 ||
    !std::isfinite(parameters_.minimum_outlier_window) ||
    parameters_.minimum_outlier_window < 0.0)
  {
    throw std::invalid_argument("Invalid object depth estimator parameters");
  }
}

DepthEstimate ObjectDepthEstimator::estimate(
  const BoundingBox2D & bounding_box,
  const std::vector<ProjectedPoint> & projected_points) const
{
  if (!bounding_box.valid()) {
    return {};
  }

  const double center_x = 0.5 * (bounding_box.x_min + bounding_box.x_max);
  const double center_y = 0.5 * (bounding_box.y_min + bounding_box.y_max);
  const double half_width =
    0.5 * (bounding_box.x_max - bounding_box.x_min) * parameters_.bbox_shrink_ratio;
  const double half_height =
    0.5 * (bounding_box.y_max - bounding_box.y_min) * parameters_.bbox_shrink_ratio;

  std::vector<const ProjectedPoint *> candidates;
  for (const ProjectedPoint & point : projected_points) {
    if (!std::isfinite(point.u) || !std::isfinite(point.v) ||
      !std::isfinite(point.depth) || !point.camera_point.allFinite())
    {
      continue;
    }
    if (point.u >= center_x - half_width && point.u <= center_x + half_width &&
      point.v >= center_y - half_height && point.v <= center_y + half_height &&
      point.depth >= parameters_.min_depth && point.depth <= parameters_.max_depth)
    {
      candidates.push_back(&point);
    }
  }
  if (candidates.size() < parameters_.min_lidar_points) {
    return {};
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const ProjectedPoint * left, const ProjectedPoint * right) {
      return left->depth < right->depth;
    });

  std::size_t cluster_begin = 0;
  for (std::size_t index = 1; index <= candidates.size(); ++index) {
    const bool cluster_ended = index == candidates.size() ||
      candidates[index]->depth - candidates[index - 1]->depth > parameters_.depth_cluster_gap;
    if (!cluster_ended) {
      continue;
    }
    if (index - cluster_begin < parameters_.min_lidar_points) {
      cluster_begin = index;
      continue;
    }

    const std::vector<const ProjectedPoint *> cluster(
      candidates.begin() + cluster_begin, candidates.begin() + index);
    std::vector<double> cluster_depths;
    cluster_depths.reserve(cluster.size());
    for (const ProjectedPoint * point : cluster) {
      cluster_depths.push_back(point->depth);
    }
    const double median_depth = median(cluster_depths);
    std::vector<double> deviations;
    deviations.reserve(cluster.size());
    for (const double depth : cluster_depths) {
      deviations.push_back(std::abs(depth - median_depth));
    }
    const double mad = median(std::move(deviations));
    const double threshold = std::max(
      parameters_.minimum_outlier_window,
      parameters_.outlier_threshold * 1.4826 * mad);

    std::vector<const ProjectedPoint *> filtered;
    filtered.reserve(cluster.size());
    for (const ProjectedPoint * point : cluster) {
      if (std::abs(point->depth - median_depth) <= threshold) {
        filtered.push_back(point);
      }
    }
    if (filtered.size() < parameters_.min_lidar_points) {
      cluster_begin = index;
      continue;
    }

    std::vector<double> filtered_depths;
    filtered_depths.reserve(filtered.size());
    for (const ProjectedPoint * point : filtered) {
      filtered_depths.push_back(point->depth);
    }

    DepthEstimate result;
    result.valid = true;
    result.position = medianPosition(filtered);
    result.depth = median(std::move(filtered_depths));
    result.lidar_point_count = filtered.size();
    return result;
  }
  return {};
}

const DepthEstimatorParameters & ObjectDepthEstimator::parameters() const noexcept
{
  return parameters_;
}

}  // namespace perception_core
