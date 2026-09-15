#ifndef NAVIGATION_BRIDGE__DETECTION_OBSTACLE_BRIDGE_HPP_
#define NAVIGATION_BRIDGE__DETECTION_OBSTACLE_BRIDGE_HPP_

#include <vector>

#include "fusion_interfaces/msg/fused_detection_array.hpp"

namespace navigation_bridge
{

struct ObstaclePoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

class DetectionObstacleBridge
{
public:
  static std::vector<ObstaclePoint> extractObstacles(
    const fusion_interfaces::msg::FusedDetectionArray & detections,
    double minimum_confidence,
    double maximum_range);
};

}  // namespace navigation_bridge

#endif  // NAVIGATION_BRIDGE__DETECTION_OBSTACLE_BRIDGE_HPP_
