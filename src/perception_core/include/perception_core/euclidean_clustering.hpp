#ifndef PERCEPTION_CORE__EUCLIDEAN_CLUSTERING_HPP_
#define PERCEPTION_CORE__EUCLIDEAN_CLUSTERING_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "perception_core/grid_key.hpp"

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
// its own or one of the 26 adjacent cells. The grid is one vector of
// (cell, point) pairs sorted by cell plus a hash map from cell to its slice,
// so a neighbour query is 27 O(1) lookups. Each slice keeps its unvisited
// points in front; a point joining a cluster is swapped out in O(1), so dense
// surfaces are not rescanned point after point. Building the grid is a sort;
// there is no per-cell allocation and no KD-tree dependency for a radius that
// is fixed per call.
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

  struct CellSegment
  {
    std::size_t begin;
    std::size_t unvisited_end;  // [begin, unvisited_end) not yet in a cluster
  };
  static constexpr std::size_t kNoCell = std::numeric_limits<std::size_t>::max();

  std::size_t cellIndex(std::uint64_t key) const;
  void markVisited(std::size_t point);
  const std::array<std::size_t, 27> & neighbourCells(std::size_t cell, std::size_t point);

  ClusteringParameters parameters_;
  std::vector<CellEntry> cells_;  // grouped by cell key
  std::unordered_map<std::uint64_t, std::size_t, GridKeyHash> cell_of_key_;
  std::vector<CellSegment> segments_;
  std::vector<std::size_t> cell_of_point_;
  std::vector<std::size_t> slot_of_point_;  // position of each point in cells_
  std::vector<std::array<std::size_t, 27>> neighbours_;  // per cell, filled lazily
  std::vector<std::uint8_t> neighbours_known_;
  std::vector<std::int64_t> coordinates_;  // 3 grid coordinates per point
  std::vector<std::uint8_t> visited_;
  std::vector<std::size_t> frontier_;
  std::vector<Cluster> clusters_;
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__EUCLIDEAN_CLUSTERING_HPP_
