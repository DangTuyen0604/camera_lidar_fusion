#include "perception_core/euclidean_clustering.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace perception_core
{

EuclideanClusterer::EuclideanClusterer(ClusteringParameters parameters)
: parameters_(std::move(parameters))
{
  if (!(parameters_.tolerance > 0.0) || !std::isfinite(parameters_.tolerance) ||
    parameters_.min_points < 1 || parameters_.max_points < parameters_.min_points)
  {
    throw std::invalid_argument("Invalid clustering parameters");
  }
}

std::uint64_t EuclideanClusterer::cellKey(
  std::int64_t x, std::int64_t y, std::int64_t z) const
{
  // 21 bits per axis: +-1M cells, i.e. +-500 km at 0.5 m; wraps harmlessly
  // beyond that because neighbours are confirmed by real distance.
  constexpr std::uint64_t kMask = (1ULL << 21) - 1;
  return ((static_cast<std::uint64_t>(x) & kMask) << 42) |
         ((static_cast<std::uint64_t>(y) & kMask) << 21) |
         (static_cast<std::uint64_t>(z) & kMask);
}

std::pair<std::size_t, std::size_t> EuclideanClusterer::cellRange(std::uint64_t key) const
{
  const auto lower = std::lower_bound(
    cells_.begin(), cells_.end(), key,
    [](const CellEntry & entry, std::uint64_t value) {return entry.key < value;});
  auto upper = lower;
  while (upper != cells_.end() && upper->key == key) {
    ++upper;
  }
  return {static_cast<std::size_t>(lower - cells_.begin()),
    static_cast<std::size_t>(upper - cells_.begin())};
}

const std::vector<Cluster> & EuclideanClusterer::cluster(
  const std::vector<PointXYZI> & points)
{
  clusters_.clear();
  const std::size_t count = points.size();
  if (count == 0) {
    return clusters_;
  }

  const double cell = parameters_.tolerance;
  const double tolerance_sq = parameters_.tolerance * parameters_.tolerance;
  coordinates_.resize(3 * count);
  cells_.resize(count);
  for (std::size_t index = 0; index < count; ++index) {
    const Eigen::Vector3d & p = points[index].position;
    std::int64_t * c = &coordinates_[3 * index];
    c[0] = static_cast<std::int64_t>(std::floor(p.x() / cell));
    c[1] = static_cast<std::int64_t>(std::floor(p.y() / cell));
    c[2] = static_cast<std::int64_t>(std::floor(p.z() / cell));
    cells_[index] = {cellKey(c[0], c[1], c[2]), index};
  }
  std::sort(
    cells_.begin(), cells_.end(),
    [](const CellEntry & a, const CellEntry & b) {
      return a.key < b.key || (a.key == b.key && a.index < b.index);
    });

  visited_.assign(count, 0);
  for (std::size_t seed = 0; seed < count; ++seed) {
    if (visited_[seed]) {
      continue;
    }
    visited_[seed] = 1;
    frontier_.clear();
    frontier_.push_back(seed);
    // frontier_ doubles as the member list: points are appended once, when
    // first reached, and the cursor walks it breadth-first.
    for (std::size_t cursor = 0; cursor < frontier_.size(); ++cursor) {
      const std::size_t current = frontier_[cursor];
      const Eigen::Vector3d & p = points[current].position;
      const std::int64_t * c = &coordinates_[3 * current];
      for (std::int64_t dx = -1; dx <= 1; ++dx) {
        for (std::int64_t dy = -1; dy <= 1; ++dy) {
          for (std::int64_t dz = -1; dz <= 1; ++dz) {
            const auto [begin, end] = cellRange(cellKey(c[0] + dx, c[1] + dy, c[2] + dz));
            for (std::size_t slot = begin; slot < end; ++slot) {
              const std::size_t neighbour = cells_[slot].index;
              if (!visited_[neighbour] &&
                (points[neighbour].position - p).squaredNorm() <= tolerance_sq)
              {
                visited_[neighbour] = 1;
                frontier_.push_back(neighbour);
              }
            }
          }
        }
      }
    }

    if (frontier_.size() < parameters_.min_points || frontier_.size() > parameters_.max_points) {
      continue;
    }
    Cluster output;
    output.indices.assign(frontier_.begin(), frontier_.end());
    std::sort(output.indices.begin(), output.indices.end());
    output.minimum = output.maximum = points[output.indices.front()].position;
    for (const std::size_t index : output.indices) {
      const Eigen::Vector3d & q = points[index].position;
      output.minimum = output.minimum.cwiseMin(q);
      output.maximum = output.maximum.cwiseMax(q);
      output.centroid += q;
    }
    output.centroid /= static_cast<double>(output.indices.size());
    clusters_.push_back(std::move(output));
  }
  return clusters_;
}

const ClusteringParameters & EuclideanClusterer::parameters() const noexcept
{
  return parameters_;
}

}  // namespace perception_core
