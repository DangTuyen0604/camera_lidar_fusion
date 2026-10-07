#ifndef PERCEPTION_CORE__EUCLIDEAN_CLUSTERING_HPP_
#define PERCEPTION_CORE__EUCLIDEAN_CLUSTERING_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "perception_core/types.hpp"

namespace perception_core
{

struct ClusteringParameters
{
  double tolerance{0.5};  // max gap between neighbouring points of one cluster [m]
  std::size_t min_points{5};
  std::size_t max_points{std::numeric_limits<std::size_t>::max()};
};

struct Cluster
{
  std::vector<std::size_t> indices;  // into the clustered point vector
  Eigen::Vector3d minimum{Eigen::Vector3d::Zero()};
  Eigen::Vector3d maximum{Eigen::Vector3d::Zero()};
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
};

// Region growing over a fixed-radius neighbourhood. Neighbours come from a
// uniform grid with cell size == tolerance: every neighbour of a point lies in
// its own or one of the 26 adjacent cells. The grid is one sorted vector of
// (cell, point) pairs, so building it is a sort and finding a cell a binary
// search: O(n log n) overall, no per-cell allocation, and no KD-tree
// dependency for a radius that is fixed per call.
class EuclideanClusterer
{
public:
  explicit EuclideanClusterer(ClusteringParameters parameters = ClusteringParameters{});

  // Clusters are ordered by their lowest point index. The returned reference
  // stays valid until the next call; all buffers are reused between calls.
  const std::vector<Cluster> & cluster(const std::vector<PointXYZI> & points);

  const ClusteringParameters & parameters() const noexcept;

private:
  struct CellEntry
  {
    std::uint64_t key;
    std::size_t index;
  };

  std::uint64_t cellKey(std::int64_t x, std::int64_t y, std::int64_t z) const;
  std::pair<std::size_t, std::size_t> cellRange(std::uint64_t key) const;

  ClusteringParameters parameters_;
  std::vector<CellEntry> cells_;           // sorted by key
  std::vector<std::int64_t> coordinates_;  // 3 grid coordinates per point
  std::vector<std::uint8_t> visited_;
  std::vector<std::size_t> frontier_;
  std::vector<Cluster> clusters_;
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__EUCLIDEAN_CLUSTERING_HPP_
