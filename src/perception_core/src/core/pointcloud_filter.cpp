#include "perception_core/pointcloud_filter.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace perception_core
{
namespace
{

struct VoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto hash_x = std::hash<std::int64_t>{}(key.x);
    const auto hash_y = std::hash<std::int64_t>{}(key.y);
    const auto hash_z = std::hash<std::int64_t>{}(key.z);
    return hash_x ^ (hash_y << 1U) ^ (hash_z << 2U);
  }
};

}  // namespace

PointCloudFilter::PointCloudFilter(PointCloudFilterParameters parameters)
: parameters_(std::move(parameters))
{
  if (!parameters_.minimum.allFinite() || !parameters_.maximum.allFinite() ||
    (parameters_.maximum.array() <= parameters_.minimum.array()).any() ||
    !std::isfinite(parameters_.minimum_intensity) ||
    !std::isfinite(parameters_.voxel_size) || parameters_.voxel_size < 0.0)
  {
    throw std::invalid_argument("Invalid point cloud filter parameters");
  }
}

std::vector<PointXYZI> PointCloudFilter::filter(
  const std::vector<PointXYZI> & points) const
{
  std::vector<PointXYZI> output;
  output.reserve(points.size());
  std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> voxels;

  for (const auto & point : points) {
    if (!point.position.allFinite() || !std::isfinite(point.intensity) ||
      point.intensity < parameters_.minimum_intensity ||
      (point.position.array() < parameters_.minimum.array()).any() ||
      (point.position.array() > parameters_.maximum.array()).any())
    {
      continue;
    }

    if (parameters_.voxel_size == 0.0) {
      output.push_back(point);
      continue;
    }

    const VoxelKey key{
      static_cast<std::int64_t>(std::floor(point.position.x() / parameters_.voxel_size)),
      static_cast<std::int64_t>(std::floor(point.position.y() / parameters_.voxel_size)),
      static_cast<std::int64_t>(std::floor(point.position.z() / parameters_.voxel_size))};
    const auto [iterator, inserted] = voxels.emplace(key, output.size());
    if (inserted) {
      output.push_back(point);
    } else if (point.intensity > output[iterator->second].intensity) {
      output[iterator->second] = point;
    }
  }
  return output;
}

const PointCloudFilterParameters & PointCloudFilter::parameters() const noexcept
{
  return parameters_;
}

}  // namespace perception_core
