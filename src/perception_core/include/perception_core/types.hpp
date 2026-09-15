#ifndef PERCEPTION_CORE__TYPES_HPP_
#define PERCEPTION_CORE__TYPES_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace perception_core
{

struct PointXYZI
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double intensity{0.0};
};

struct CalibrationData
{
  int image_width{0};
  int image_height{0};
  Eigen::Matrix3d camera_matrix{Eigen::Matrix3d::Identity()};
  Eigen::Matrix<double, 3, 4> projection_matrix{
    Eigen::Matrix<double, 3, 4>::Zero()};
  Eigen::Matrix4d rectification_matrix{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d lidar_to_camera{Eigen::Matrix4d::Identity()};
  std::string lidar_frame{"velodyne"};
  std::string camera_frame{"camera_optical_frame"};
};

struct ProjectedPoint
{
  double u{0.0};
  double v{0.0};
  Eigen::Vector3d camera_point{Eigen::Vector3d::Zero()};
  double depth{0.0};
  double intensity{0.0};
};

struct BoundingBox2D
{
  double x_min{0.0};
  double y_min{0.0};
  double x_max{0.0};
  double y_max{0.0};

  bool valid() const
  {
    return x_max > x_min && y_max > y_min;
  }
};

struct DepthEstimate
{
  bool valid{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double depth{0.0};
  std::size_t lidar_point_count{0};
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__TYPES_HPP_
