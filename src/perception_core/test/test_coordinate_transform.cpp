#include <cmath>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "perception_core/coordinate_transform.hpp"

namespace
{

TEST(CoordinateTransform, IdentityLeavesPointUnchanged)
{
  const Eigen::Vector3d point(1.0, -2.0, 3.0);
  EXPECT_TRUE(perception_core::CoordinateTransform::transformPoint(
      Eigen::Matrix4d::Identity(), point).isApprox(point));
}

TEST(CoordinateTransform, AppliesRotationAndTranslation)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.topLeftCorner<3, 3>() =
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform.topRightCorner<3, 1>() = Eigen::Vector3d(1.0, 2.0, 3.0);

  const Eigen::Vector3d result =
    perception_core::CoordinateTransform::transformPoint(
    transform, Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_TRUE(result.isApprox(Eigen::Vector3d(1.0, 3.0, 3.0), 1.0e-12));
}

TEST(CoordinateTransform, InverseRecoversOriginalPoint)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.topLeftCorner<3, 3>() =
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
  transform.topRightCorner<3, 1>() = Eigen::Vector3d(4.0, -1.0, 2.0);
  const Eigen::Vector3d point(3.0, 2.0, 9.0);

  const auto transformed =
    perception_core::CoordinateTransform::transformPoint(transform, point);
  const auto recovered = perception_core::CoordinateTransform::transformPoint(
    perception_core::CoordinateTransform::inverse(transform), transformed);
  EXPECT_TRUE(recovered.isApprox(point, 1.0e-12));
}

TEST(CoordinateTransform, RejectsNonRigidMatrix)
{
  Eigen::Matrix4d scale = Eigen::Matrix4d::Identity();
  scale(0, 0) = 2.0;
  EXPECT_FALSE(perception_core::CoordinateTransform::isRigidTransform(scale));
  EXPECT_THROW(
    perception_core::CoordinateTransform::inverse(scale), std::invalid_argument);
}

}  // namespace
