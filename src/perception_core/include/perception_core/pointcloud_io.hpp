#ifndef PERCEPTION_CORE__POINTCLOUD_IO_HPP_
#define PERCEPTION_CORE__POINTCLOUD_IO_HPP_

#include <cstddef>
#include <vector>

#include "perception_core/types.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

namespace perception_core
{

struct PointCloudReadStats
{
  std::size_t total{0};
  std::size_t non_finite{0};
};

// Read x, y, z and an optional intensity field into `points`, reusing its
// capacity between calls. Every numeric PointField datatype and both byte
// orders are accepted, so drivers that publish uint8/uint16 intensity
// (Hesai, Robosense) work like float32 ones (Velodyne, Ouster, KITTI).
// row_step is honoured for padded organized clouds. Points with a non-finite
// coordinate or intensity are skipped and counted. Throws
// std::invalid_argument for a layout that does not fit its data buffer.
PointCloudReadStats readPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  std::vector<PointXYZI> & points);

std::vector<PointXYZI> readPointCloud(const sensor_msgs::msg::PointCloud2 & cloud);

// Write an unorganized FLOAT32 x/y/z/intensity cloud, reusing cloud.data.
void writePointCloud(
  const std::vector<PointXYZI> & points,
  const std_msgs::msg::Header & header,
  sensor_msgs::msg::PointCloud2 & cloud);

}  // namespace perception_core

#endif  // PERCEPTION_CORE__POINTCLOUD_IO_HPP_
