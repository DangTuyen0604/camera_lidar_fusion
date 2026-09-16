#include "perception_core/calibration_loader.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <yaml-cpp/yaml.h>

#include "perception_core/coordinate_transform.hpp"

namespace perception_core
{
namespace
{

template<int Rows, int Cols>
Eigen::Matrix<double, Rows, Cols> readMatrix(
  const YAML::Node & root,
  const std::string & key)
{
  const YAML::Node section = root[key];
  if (!section || !section.IsMap() || !section["rows"] || !section["cols"] ||
    !section["data"] || !section["data"].IsSequence())
  {
    throw std::runtime_error("Invalid or missing YAML matrix: " + key);
  }
  if (section["rows"].as<int>() != Rows || section["cols"].as<int>() != Cols ||
    section["data"].size() != static_cast<std::size_t>(Rows * Cols))
  {
    throw std::runtime_error(
            key + " must have shape " + std::to_string(Rows) + "x" +
            std::to_string(Cols));
  }

  Eigen::Matrix<double, Rows, Cols> result;
  for (int row = 0; row < Rows; ++row) {
    for (int column = 0; column < Cols; ++column) {
      const double value = section["data"][row * Cols + column].as<double>();
      if (!std::isfinite(value)) {
        throw std::runtime_error(key + " must contain finite values");
      }
      result(row, column) = value;
    }
  }
  return result;
}

double readFiniteScalar(const YAML::Node & section, const std::string & key)
{
  if (!section || !section.IsMap() || !section[key]) {
    throw std::runtime_error("Missing calibration value: " + key);
  }
  const double value = section[key].as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error("Calibration value must be finite: " + key);
  }
  return value;
}

YAML::Node loadYaml(const std::string & path)
{
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("Calibration YAML not found: " + path);
  }
  try {
    const YAML::Node root = YAML::LoadFile(path);
    if (!root || !root.IsMap()) {
      throw std::runtime_error("Calibration YAML must contain a mapping: " + path);
    }
    return root;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("Invalid calibration YAML " + path + ": " + error.what());
  }
}

std::unordered_map<std::string, std::vector<double>> loadKittiValues(
  const std::string & path)
{
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("KITTI calibration not found: " + path);
  }
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("Cannot open KITTI calibration: " + path);
  }

  std::unordered_map<std::string, std::vector<double>> values;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(stream, line)) {
    ++line_number;
    const auto separator = line.find(':');
    if (separator == std::string::npos) {
      if (line.find_first_not_of(" \t\r") == std::string::npos) {
        continue;
      }
      throw std::runtime_error(
              "Invalid KITTI calibration line " + std::to_string(line_number));
    }
    const std::string key = line.substr(0, separator);
    std::istringstream row(line.substr(separator + 1));
    std::vector<double> parsed;
    double value = 0.0;
    while (row >> value) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("KITTI calibration contains a non-finite value: " + key);
      }
      parsed.push_back(value);
    }
    if (!row.eof()) {
      throw std::runtime_error("Invalid numeric value in KITTI calibration: " + key);
    }
    if (!values.emplace(key, std::move(parsed)).second) {
      throw std::runtime_error("Duplicate KITTI calibration key: " + key);
    }
  }
  return values;
}

const std::vector<double> & requireKittiValues(
  const std::unordered_map<std::string, std::vector<double>> & values,
  const std::vector<std::string> & keys,
  std::size_t expected_size)
{
  for (const auto & key : keys) {
    const auto iterator = values.find(key);
    if (iterator == values.end()) {
      continue;
    }
    if (iterator->second.size() != expected_size) {
      throw std::runtime_error(
              "KITTI calibration key " + key + " must contain " +
              std::to_string(expected_size) + " values");
    }
    return iterator->second;
  }
  throw std::runtime_error("Missing KITTI calibration key: " + keys.front());
}

template<int Rows, int Cols>
Eigen::Matrix<double, Rows, Cols> matrixFromRowMajor(
  const std::vector<double> & values)
{
  Eigen::Matrix<double, Rows, Cols> result;
  for (int row = 0; row < Rows; ++row) {
    for (int column = 0; column < Cols; ++column) {
      result(row, column) = values[static_cast<std::size_t>(row * Cols + column)];
    }
  }
  return result;
}

void validateCalibration(const CalibrationData & calibration)
{
  if (calibration.image_width <= 0 || calibration.image_height <= 0) {
    throw std::runtime_error("Calibration image dimensions must be positive");
  }
  if (!calibration.camera_matrix.allFinite() ||
    std::abs(calibration.camera_matrix.determinant()) < 1.0e-12)
  {
    throw std::runtime_error("Camera matrix must be finite and non-singular");
  }
  if (!calibration.projection_matrix.allFinite() ||
    std::abs(calibration.projection_matrix.leftCols<3>().determinant()) < 1.0e-12)
  {
    throw std::runtime_error("Projection matrix must be finite and non-singular");
  }
  if (!CoordinateTransform::isRigidTransform(calibration.rectification_matrix) ||
    !CoordinateTransform::isRigidTransform(calibration.lidar_to_camera))
  {
    throw std::runtime_error("Calibration transforms must be rigid homogeneous transforms");
  }
  if (calibration.lidar_frame.empty() || calibration.camera_frame.empty() ||
    calibration.lidar_frame == calibration.camera_frame)
  {
    throw std::runtime_error("Calibration frames must be non-empty and distinct");
  }
}

}  // namespace

CalibrationData CalibrationLoader::loadFromYaml(
  const std::string & intrinsics_path,
  const std::string & extrinsics_path,
  const std::string & expected_lidar_frame,
  const std::string & expected_camera_frame)
{
  const YAML::Node intrinsics = loadYaml(intrinsics_path);
  const YAML::Node extrinsics = loadYaml(extrinsics_path);

  CalibrationData result;
  result.image_width = intrinsics["image_width"].as<int>(0);
  result.image_height = intrinsics["image_height"].as<int>(0);
  if (result.image_width <= 0 || result.image_height <= 0) {
    throw std::runtime_error("Calibration image dimensions must be positive");
  }
  result.camera_matrix = readMatrix<3, 3>(intrinsics, "camera_matrix");
  result.projection_matrix = readMatrix<3, 4>(intrinsics, "projection_matrix");
  result.rectification_matrix = Eigen::Matrix4d::Identity();
  result.rectification_matrix.topLeftCorner<3, 3>() =
    readMatrix<3, 3>(intrinsics, "rectification_matrix");

  result.lidar_frame = extrinsics["parent_frame"].as<std::string>("");
  result.camera_frame = extrinsics["child_frame"].as<std::string>("");
  const std::string convention = extrinsics["convention"].as<std::string>("");
  if (result.lidar_frame != expected_lidar_frame) {
    throw std::runtime_error(
            "Extrinsic parent frame mismatch: expected=" + expected_lidar_frame +
            ", actual=" + result.lidar_frame);
  }
  if (result.camera_frame != expected_camera_frame) {
    throw std::runtime_error(
            "Extrinsic child frame mismatch: expected=" + expected_camera_frame +
            ", actual=" + result.camera_frame);
  }
  if (convention != "child_pose_in_parent") {
    throw std::runtime_error("Extrinsic convention must be child_pose_in_parent");
  }

  const YAML::Node translation_node = extrinsics["translation"];
  const YAML::Node rotation_node = extrinsics["rotation"];
  const Eigen::Vector3d translation(
    readFiniteScalar(translation_node, "x"),
    readFiniteScalar(translation_node, "y"),
    readFiniteScalar(translation_node, "z"));
  Eigen::Quaterniond quaternion(
    readFiniteScalar(rotation_node, "w"),
    readFiniteScalar(rotation_node, "x"),
    readFiniteScalar(rotation_node, "y"),
    readFiniteScalar(rotation_node, "z"));
  if (quaternion.norm() < 1.0e-12) {
    throw std::runtime_error("Extrinsic quaternion norm must be greater than zero");
  }
  quaternion.normalize();

  Eigen::Matrix4d lidar_from_camera = Eigen::Matrix4d::Identity();
  lidar_from_camera.topLeftCorner<3, 3>() = quaternion.toRotationMatrix();
  lidar_from_camera.topRightCorner<3, 1>() = translation;
  result.lidar_to_camera = CoordinateTransform::inverse(lidar_from_camera);
  validateCalibration(result);
  return result;
}

CalibrationData CalibrationLoader::loadFromKitti(
  const std::string & calibration_path,
  int image_width,
  int image_height,
  const std::string & lidar_frame,
  const std::string & camera_frame)
{
  const auto values = loadKittiValues(calibration_path);
  CalibrationData result;
  result.image_width = image_width;
  result.image_height = image_height;
  result.lidar_frame = lidar_frame;
  result.camera_frame = camera_frame;
  result.projection_matrix = matrixFromRowMajor<3, 4>(
    requireKittiValues(values, {"P2", "P_rect_02"}, 12));
  result.camera_matrix = result.projection_matrix.leftCols<3>();
  result.rectification_matrix = Eigen::Matrix4d::Identity();
  result.rectification_matrix.topLeftCorner<3, 3>() = matrixFromRowMajor<3, 3>(
    requireKittiValues(values, {"R0_rect", "R_rect_00"}, 9));
  result.lidar_to_camera = Eigen::Matrix4d::Identity();
  result.lidar_to_camera.topRows<3>() = matrixFromRowMajor<3, 4>(
    requireKittiValues(values, {"Tr_velo_to_cam", "Tr"}, 12));
  validateCalibration(result);
  return result;
}

}  // namespace perception_core
