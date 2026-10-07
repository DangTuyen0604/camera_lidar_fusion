#include "perception_core/euclidean_clustering.hpp"

#include <algorithm>
#include <array>
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

std::size_t EuclideanClusterer::cellIndex(std::uint64_t key) const
{
  const auto found = cell_of_key_.find(key);
  return found == cell_of_key_.end() ? kNoCell : found->second;
}

const std::array<std::size_t, 27> & EuclideanClusterer::neighbourCells(
  std::size_t cell, std::size_t point)
{
  // Every point of a cell shares its 27 neighbour cells: look them up once.
  if (!neighbours_known_[cell]) {
    const std::int64_t * c = &coordinates_[3 * point];
    std::size_t slot = 0;
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        for (std::int64_t dz = -1; dz <= 1; ++dz) {
          neighbours_[cell][slot++] = cellIndex(packGridKey(c[0] + dx, c[1] + dy, c[2] + dz));
        }
      }
    }
    neighbours_known_[cell] = 1;
  }
  return neighbours_[cell];
}

void EuclideanClusterer::markVisited(std::size_t point)
{
  // Swap the point behind its cell's unvisited segment so later scans of
  // that cell never look at it again.
  CellSegment & segment = segments_[cell_of_point_[point]];
  const std::size_t slot = slot_of_point_[point];
  const std::size_t last = --segment.unvisited_end;
  std::swap(cells_[slot], cells_[last]);
  slot_of_point_[cells_[slot].index] = slot;
  slot_of_point_[cells_[last].index] = last;
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
    cells_[index] = {packGridKey(c[0], c[1], c[2]), index};
  }
  std::sort(
    cells_.begin(), cells_.end(),
    [](const CellEntry & a, const CellEntry & b) {
      return a.key < b.key || (a.key == b.key && a.index < b.index);
    });
  cell_of_key_.clear();
  cell_of_key_.reserve(count);
  segments_.clear();
  cell_of_point_.resize(count);
  slot_of_point_.resize(count);
  for (std::size_t begin = 0; begin < count; ) {
    std::size_t end = begin;
    while (end < count && cells_[end].key == cells_[begin].key) {
      cell_of_point_[cells_[end].index] = segments_.size();
      slot_of_point_[cells_[end].index] = end;
      ++end;
    }
    cell_of_key_.emplace(cells_[begin].key, segments_.size());
    segments_.push_back({begin, end});
    begin = end;
  }
  neighbours_.resize(segments_.size());
  neighbours_known_.assign(segments_.size(), 0);

  visited_.assign(count, 0);
  for (std::size_t seed = 0; seed < count; ++seed) {
    if (visited_[seed]) {
      continue;
    }
    visited_[seed] = 1;
    markVisited(seed);
    frontier_.clear();
    frontier_.push_back(seed);
    // frontier_ doubles as the member list: points are appended once, when
    // first reached, and the cursor walks it breadth-first.
    for (std::size_t cursor = 0; cursor < frontier_.size(); ++cursor) {
      const std::size_t current = frontier_[cursor];
      const Eigen::Vector3d & p = points[current].position;
      for (const std::size_t neighbour_cell : neighbourCells(cell_of_point_[current], current)) {
        if (neighbour_cell == kNoCell) {
          continue;
        }
        const CellSegment & segment = segments_[neighbour_cell];
        for (std::size_t slot = segment.begin; slot < segment.unvisited_end; ) {
          const std::size_t neighbour = cells_[slot].index;
          if ((points[neighbour].position - p).squaredNorm() <= tolerance_sq) {
            visited_[neighbour] = 1;
            markVisited(neighbour);  // another unvisited point moves into `slot`
            frontier_.push_back(neighbour);
          } else {
            ++slot;
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
