#ifndef PERCEPTION_CORE__LIDAR_PROJECTOR_HPP_
#define PERCEPTION_CORE__LIDAR_PROJECTOR_HPP_

#include <vector>

#include "perception_core/types.hpp"

namespace perception_core
{

class LidarProjector
{
public:
  explicit LidarProjector(CalibrationData calibration);

  std::vector<ProjectedPoint> project(
    const std::vector<PointXYZI> & lidar_points,
    int image_width = 0,
    int image_height = 0,
    double min_depth = 0.1,
    double max_depth = 0.0) const;

  const CalibrationData & calibration() const noexcept;

private:
  CalibrationData calibration_;
};

}  // namespace perception_core

#endif  // PERCEPTION_CORE__LIDAR_PROJECTOR_HPP_
