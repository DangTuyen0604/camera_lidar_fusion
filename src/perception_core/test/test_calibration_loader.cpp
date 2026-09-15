#include <string>

#include <gtest/gtest.h>

#include "perception_core/calibration_loader.hpp"
#include "perception_core/coordinate_transform.hpp"

namespace
{

std::string testFile(const std::string & name)
{
  return std::string(TEST_DATA_DIR) + "/" + name;
}

TEST(CalibrationLoader, LoadsAndConvertsChildPoseToLidarToCamera)
{
  const auto calibration = perception_core::CalibrationLoader::loadFromYaml(
    testFile("valid_intrinsics.yaml"), testFile("valid_extrinsics.yaml"));

  EXPECT_EQ(calibration.image_width, 100);
  EXPECT_EQ(calibration.image_height, 80);
  EXPECT_DOUBLE_EQ(calibration.camera_matrix(0, 0), 100.0);
  EXPECT_DOUBLE_EQ(calibration.projection_matrix(0, 2), 50.0);
  EXPECT_EQ(calibration.lidar_frame, "velodyne");
  EXPECT_EQ(calibration.camera_frame, "camera_optical_frame");

  const Eigen::Vector3d camera_origin_in_lidar(1.0, 2.0, 3.0);
  const Eigen::Vector3d camera_origin =
    perception_core::CoordinateTransform::transformPoint(
    calibration.lidar_to_camera, camera_origin_in_lidar);
  EXPECT_TRUE(camera_origin.isApprox(Eigen::Vector3d::Zero(), 1.0e-12));
}

TEST(CalibrationLoader, RejectsMissingFile)
{
  EXPECT_THROW(
    perception_core::CalibrationLoader::loadFromYaml(
      "/does/not/exist.yaml", testFile("valid_extrinsics.yaml")),
    std::runtime_error);
}

TEST(CalibrationLoader, RejectsInvalidIntrinsics)
{
  EXPECT_THROW(
    perception_core::CalibrationLoader::loadFromYaml(
      testFile("invalid_calibration.yaml"), testFile("valid_extrinsics.yaml")),
    std::runtime_error);
}

TEST(CalibrationLoader, RejectsUnexpectedFrame)
{
  EXPECT_THROW(
    perception_core::CalibrationLoader::loadFromYaml(
      testFile("valid_intrinsics.yaml"), testFile("valid_extrinsics.yaml"),
      "wrong_lidar_frame", "camera_optical_frame"),
    std::runtime_error);
}

TEST(CalibrationLoader, LoadsRepositoryKittiCalibration)
{
  const auto calibration = perception_core::CalibrationLoader::loadFromYaml(
    std::string(FUSION_CONFIG_DIR) + "/camera_intrinsics.yaml",
    std::string(FUSION_CONFIG_DIR) + "/lidar_camera_extrinsics.yaml");

  EXPECT_EQ(calibration.image_width, 1242);
  EXPECT_EQ(calibration.image_height, 375);
  EXPECT_TRUE(perception_core::CoordinateTransform::isRigidTransform(
      calibration.lidar_to_camera));
  EXPECT_TRUE(calibration.projection_matrix.allFinite());
}

}  // namespace
