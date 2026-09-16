#include "perception_core/lidar_projector.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <Eigen/LU>

#include "perception_core/coordinate_transform.hpp"

namespace perception_core
{

LidarProjector::LidarProjector(CalibrationData calibration)
: calibration_(std::move(calibration))
{
  if (calibration_.image_width <= 0 || calibration_.image_height <= 0 ||
    !calibration_.projection_matrix.allFinite() ||
    std::abs(calibration_.projection_matrix.leftCols<3>().determinant()) < 1.0e-12 ||
    !CoordinateTransform::isRigidTransform(calibration_.rectification_matrix) ||
    !CoordinateTransform::isRigidTransform(calibration_.lidar_to_camera))
  {
    throw std::invalid_argument("LidarProjector received invalid calibration");
  }
}

std::vector<ProjectedPoint> LidarProjector::project(
  const std::vector<PointXYZI> & lidar_points,
  int image_width,
  int image_height,
  double min_depth,
  double max_depth) const
{
  const int width = image_width > 0 ? image_width : calibration_.image_width;
  const int height = image_height > 0 ? image_height : calibration_.image_height;
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("Image dimensions must be positive");
  }
  if (!std::isfinite(min_depth) || min_depth <= 0.0 ||
    !std::isfinite(max_depth) || (max_depth > 0.0 && max_depth <= min_depth))
  {
    throw std::invalid_argument("Invalid projection depth range");
  }

  std::vector<ProjectedPoint> output;
  output.reserve(lidar_points.size());
  const Eigen::Matrix4d rectified_from_lidar =
    calibration_.rectification_matrix * calibration_.lidar_to_camera;

  for (const PointXYZI & point : lidar_points) {
    if (!point.position.allFinite() || !std::isfinite(point.intensity)) {
      continue;
    }
    const Eigen::Vector4d homogeneous(
      point.position.x(), point.position.y(), point.position.z(), 1.0);
    const Eigen::Vector4d camera_homogeneous = rectified_from_lidar * homogeneous;
    const Eigen::Vector3d camera_point = camera_homogeneous.head<3>();
    if (!camera_homogeneous.allFinite() ||
      std::abs(camera_homogeneous.w()) < 1.0e-12)
    {
      continue;
    }
    const Eigen::Vector3d normalized_camera_point = camera_point / camera_homogeneous.w();
    const double normalized_depth = normalized_camera_point.z();
    if (!normalized_camera_point.allFinite() || normalized_depth < min_depth ||
      (max_depth > 0.0 && normalized_depth > max_depth))
    {
      continue;
    }

    const Eigen::Vector3d pixel_homogeneous =
      calibration_.projection_matrix * camera_homogeneous;
    if (!pixel_homogeneous.allFinite() || std::abs(pixel_homogeneous.z()) < 1.0e-12) {
      continue;
    }
    const double u = pixel_homogeneous.x() / pixel_homogeneous.z();
    const double v = pixel_homogeneous.y() / pixel_homogeneous.z();
    if (!std::isfinite(u) || !std::isfinite(v) ||
      u < 0.0 || u >= static_cast<double>(width) ||
      v < 0.0 || v >= static_cast<double>(height))
    {
      continue;
    }
    output.push_back(ProjectedPoint{
        u, v, normalized_camera_point, normalized_depth, point.intensity});
  }
  return output;
}

const CalibrationData & LidarProjector::calibration() const noexcept
{
  return calibration_;
}

}  // namespace perception_core
