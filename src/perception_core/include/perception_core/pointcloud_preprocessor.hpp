#ifndef PERCEPTION_CORE__POINTCLOUD_PREPROCESSOR_HPP_
#define PERCEPTION_CORE__POINTCLOUD_PREPROCESSOR_HPP_

#include <vector>

#include "perception_core/euclidean_clustering.hpp"
#include "perception_core/ground_segmentation.hpp"
#include "perception_core/pointcloud_filter.hpp"
#include "perception_core/types.hpp"

namespace perception_core
{

struct PreprocessorParameters
{
  PointCloudFilterParameters filter;
  bool remove_ground{true};
  GroundSegmentationParameters ground;
  bool cluster{true};
  ClusteringParameters clustering;
};

struct PreprocessorTiming
{
  double filter_ms{0.0};
  double ground_ms{0.0};
  double cluster_ms{0.0};
  double total_ms{0.0};
};

// Views into the preprocessor's buffers; valid until the next process() call.
struct PreprocessorOutput
{
  const std::vector<PointXYZI> * filtered{nullptr};
  const std::vector<PointXYZI> * ground{nullptr};
  const std::vector<PointXYZI> * obstacles{nullptr};
  const std::vector<Cluster> * clusters{nullptr};  // indices refer to *obstacles
  GroundPlane plane;
  PreprocessorTiming timing;
};

// crop/voxel filter -> RANSAC ground removal -> Euclidean clustering. Points
// must already be in the frame the parameters are expressed in. The point
// buffers and the clustering grid are members reused across frames; what is
// still allocated per frame is the voxel map and each cluster's index list.
class PointCloudPreprocessor
{
public:
  explicit PointCloudPreprocessor(PreprocessorParameters parameters = PreprocessorParameters{});

  PreprocessorOutput process(const std::vector<PointXYZI> & points);

  const PreprocessorParameters & parameters() const noexcept;

private:
  PreprocessorParameters parameters_;
  PointCloudFilter filter_;
  RansacGroundSegmenter ground_;
  EuclideanClusterer clusterer_;
  std::vector<PointXYZI> filtered_;
  std::vector<PointXYZI> ground_points_;
  std::vector<PointXYZI> obstacles_;
  std::vector<Cluster> no_clusters_;
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__POINTCLOUD_PREPROCESSOR_HPP_
