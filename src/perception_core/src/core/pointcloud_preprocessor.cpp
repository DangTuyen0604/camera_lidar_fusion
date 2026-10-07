#include "perception_core/pointcloud_preprocessor.hpp"

#include <chrono>
#include <utility>

namespace perception_core
{
namespace
{

using Clock = std::chrono::steady_clock;

double millisecondsSince(Clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

}  // namespace

PointCloudPreprocessor::PointCloudPreprocessor(PreprocessorParameters parameters)
: parameters_(std::move(parameters)),
  filter_(parameters_.filter),
  ground_(parameters_.ground),
  clusterer_(parameters_.clustering)
{
}

PreprocessorOutput PointCloudPreprocessor::process(const std::vector<PointXYZI> & points)
{
  PreprocessorOutput output;
  const auto start = Clock::now();

  auto stage = Clock::now();
  filter_.filter(points, filtered_);
  output.timing.filter_ms = millisecondsSince(stage);

  stage = Clock::now();
  if (parameters_.remove_ground) {
    output.plane = ground_.segment(filtered_, ground_points_, obstacles_);
  } else {
    ground_points_.clear();
    obstacles_ = filtered_;
  }
  output.timing.ground_ms = millisecondsSince(stage);

  stage = Clock::now();
  output.clusters = parameters_.cluster ? &clusterer_.cluster(obstacles_) : &no_clusters_;
  output.timing.cluster_ms = millisecondsSince(stage);

  output.filtered = &filtered_;
  output.ground = &ground_points_;
  output.obstacles = &obstacles_;
  output.timing.total_ms = millisecondsSince(start);
  return output;
}

const PreprocessorParameters & PointCloudPreprocessor::parameters() const noexcept
{
  return parameters_;
}

}  // namespace perception_core
