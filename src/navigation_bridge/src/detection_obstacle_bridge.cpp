#include "navigation_bridge/detection_obstacle_bridge.hpp"

#include <cmath>
#include <stdexcept>

namespace navigation_bridge
{

std::vector<ObstaclePoint> DetectionObstacleBridge::extractObstacles(
  const fusion_interfaces::msg::FusedDetectionArray & detections,
  double minimum_confidence,
  double maximum_range)
{
  if (!std::isfinite(minimum_confidence) || minimum_confidence < 0.0 ||
    minimum_confidence > 1.0 || !std::isfinite(maximum_range) || maximum_range <= 0.0)
  {
    throw std::invalid_argument("Invalid obstacle bridge thresholds");
  }

  std::vector<ObstaclePoint> output;
  output.reserve(detections.detections.size());
  for (const auto & detection : detections.detections) {
    const auto & position = detection.position;
    const double range = std::hypot(position.x, position.y);
    if (!detection.valid || detection.detection.confidence < minimum_confidence ||
      !std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || range > maximum_range)
    {
      continue;
    }
    output.push_back({
        static_cast<float>(position.x), static_cast<float>(position.y),
        static_cast<float>(position.z)});
  }
  return output;
}

}  // namespace navigation_bridge
