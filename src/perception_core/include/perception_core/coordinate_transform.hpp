#ifndef PERCEPTION_CORE__COORDINATE_TRANSFORM_HPP_
#define PERCEPTION_CORE__COORDINATE_TRANSFORM_HPP_

#include <Eigen/Core>

namespace perception_core
{

class CoordinateTransform
{
public:
  static Eigen::Vector3d transformPoint(
    const Eigen::Matrix4d & transform,
    const Eigen::Vector3d & point);

  static Eigen::Matrix4d inverse(const Eigen::Matrix4d & transform);

  static bool isRigidTransform(
    const Eigen::Matrix4d & transform,
    double tolerance = 1.0e-6);
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__COORDINATE_TRANSFORM_HPP_
