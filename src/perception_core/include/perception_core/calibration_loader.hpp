#ifndef PERCEPTION_CORE__CALIBRATION_LOADER_HPP_
#define PERCEPTION_CORE__CALIBRATION_LOADER_HPP_

#include <string>

#include "perception_core/types.hpp"

namespace perception_core
{

class CalibrationLoader
{
public:
  static CalibrationData loadFromYaml(
    const std::string & intrinsics_path,
    const std::string & extrinsics_path,
    const std::string & expected_lidar_frame = "velodyne",
    const std::string & expected_camera_frame = "camera_optical_frame");

  // Loads the KITTI object-detection calibration format (P2, R0_rect and
  // Tr_velo_to_cam). The image size is not stored in that format and must be
  // supplied by the caller.
  static CalibrationData loadFromKitti(
    const std::string & calibration_path,
    int image_width,
    int image_height,
    const std::string & lidar_frame = "velodyne",
    const std::string & camera_frame = "camera_optical_frame");
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__CALIBRATION_LOADER_HPP_
