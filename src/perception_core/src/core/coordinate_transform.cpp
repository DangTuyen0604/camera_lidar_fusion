#include "perception_core/coordinate_transform.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace perception_core
{

Eigen::Vector3d CoordinateTransform::transformPoint(
  const Eigen::Matrix4d & transform,
  const Eigen::Vector3d & point)
{
  if (!transform.allFinite() || !point.allFinite()) {
    throw std::invalid_argument("Transform and point must contain finite values");
  }

  const Eigen::Vector4d homogeneous(point.x(), point.y(), point.z(), 1.0);
  const Eigen::Vector4d transformed = transform * homogeneous;
  if (!transformed.allFinite() || std::abs(transformed.w()) < 1.0e-12) {
    throw std::runtime_error("Point transformation produced an invalid result");
  }
  return transformed.head<3>() / transformed.w();
}

Eigen::Matrix4d CoordinateTransform::inverse(const Eigen::Matrix4d & transform)
{
  if (!isRigidTransform(transform)) {
    throw std::invalid_argument("Matrix is not a valid rigid transform");
  }

  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d rotation = transform.topLeftCorner<3, 3>();
  result.topLeftCorner<3, 3>() = rotation.transpose();
  result.topRightCorner<3, 1>() =
    -rotation.transpose() * transform.topRightCorner<3, 1>();
  return result;
}

bool CoordinateTransform::isRigidTransform(
  const Eigen::Matrix4d & transform,
  double tolerance)
{
  if (tolerance <= 0.0 || !transform.allFinite()) {
    return false;
  }
  const Eigen::RowVector4d expected_last_row(0.0, 0.0, 0.0, 1.0);
  if (!transform.row(3).isApprox(expected_last_row, tolerance)) {
    return false;
  }

  const Eigen::Matrix3d rotation = transform.topLeftCorner<3, 3>();
  return (rotation.transpose() * rotation).isApprox(
    Eigen::Matrix3d::Identity(), tolerance) &&
         std::abs(rotation.determinant() - 1.0) <= tolerance;
}

}  // namespace perception_core
