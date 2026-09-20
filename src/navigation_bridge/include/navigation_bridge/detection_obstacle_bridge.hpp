#ifndef NAVIGATION_BRIDGE__DETECTION_OBSTACLE_BRIDGE_HPP_
#define NAVIGATION_BRIDGE__DETECTION_OBSTACLE_BRIDGE_HPP_

#include <cstdint>
#include <string>
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

struct BridgeConfig
{
  double minimum_confidence{0.35};
  double minimum_range{0.20};
  double maximum_range{30.0};
  double footprint_resolution{0.10};
};

struct RejectionCounters
{
  std::uint64_t accepted{0};
  std::uint64_t invalid{0};
  std::uint64_t confidence{0};
  std::uint64_t range{0};
  std::uint64_t non_finite{0};
};

class DetectionObstacleBridge
{
public:
  static std::vector<ObstaclePoint> extractObstacles(
    const fusion_interfaces::msg::FusedDetectionArray & detections,
    double minimum_confidence,
    double maximum_range);

  static std::vector<ObstaclePoint> extractFootprints(
    const fusion_interfaces::msg::FusedDetectionArray & detections,
    const BridgeConfig & config,
    RejectionCounters * counters = nullptr);

  static std::pair<double, double> footprintSize(const std::string & class_name);
};

}  // namespace navigation_bridge

#endif  // NAVIGATION_BRIDGE__DETECTION_OBSTACLE_BRIDGE_HPP_
