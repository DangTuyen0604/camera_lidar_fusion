#include "navigation_bridge/detection_obstacle_bridge.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

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

std::pair<double, double> DetectionObstacleBridge::footprintSize(
  const std::string & class_name)
{
  if (class_name == "person" || class_name == "worker") {
    return {0.60, 0.60};
  }
  if (class_name == "pallet") {
    return {1.20, 0.80};
  }
  if (class_name == "box" || class_name == "cargo_box") {
    return {0.60, 0.60};
  }
  return {0.40, 0.40};
}

std::vector<ObstaclePoint> DetectionObstacleBridge::extractFootprints(
  const fusion_interfaces::msg::FusedDetectionArray & detections,
  const BridgeConfig & config,
  RejectionCounters * counters)
{
  if (!std::isfinite(config.minimum_confidence) || config.minimum_confidence < 0.0 ||
    config.minimum_confidence > 1.0 || !std::isfinite(config.minimum_range) ||
    config.minimum_range < 0.0 || !std::isfinite(config.maximum_range) ||
    config.maximum_range <= config.minimum_range ||
    !std::isfinite(config.footprint_resolution) || config.footprint_resolution <= 0.0)
  {
    throw std::invalid_argument("Invalid obstacle bridge configuration");
  }

  RejectionCounters local;
  auto & count = counters == nullptr ? local : *counters;
  std::vector<ObstaclePoint> output;
  for (const auto & detection : detections.detections) {
    const auto & p = detection.position;
    if (!detection.valid) {
      ++count.invalid;
      continue;
    }
    if (!std::isfinite(detection.detection.confidence) ||
      detection.detection.confidence < config.minimum_confidence)
    {
      ++count.confidence;
      continue;
    }
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      ++count.non_finite;
      continue;
    }
    const double range = std::hypot(p.x, p.y);
    if (range < config.minimum_range || range > config.maximum_range) {
      ++count.range;
      continue;
    }

    const auto [length, width] = footprintSize(detection.detection.class_name);
    const int nx = std::max(1, static_cast<int>(std::ceil(length / config.footprint_resolution)));
    const int ny = std::max(1, static_cast<int>(std::ceil(width / config.footprint_resolution)));
    for (int ix = 0; ix <= nx; ++ix) {
      for (int iy = 0; iy <= ny; ++iy) {
        output.push_back({
            static_cast<float>(p.x - length * 0.5 + length * ix / nx),
            static_cast<float>(p.y - width * 0.5 + width * iy / ny),
            static_cast<float>(p.z)});
      }
    }
    ++count.accepted;
  }
  return output;
}

}  // namespace navigation_bridge
