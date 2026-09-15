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
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__CALIBRATION_LOADER_HPP_
