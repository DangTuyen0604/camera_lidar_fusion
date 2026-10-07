// Offline benchmark of PointCloudPreprocessor with the node's own YAML profile.
//
//   pointcloud_preprocess_benchmark <profile.yaml> [--frames N] [--no-reuse]
//                                   [kitti_000000.bin ...]
//
// Without .bin files it ray-casts a synthetic HDL-64E-like scan (64 rings,
// 2083 azimuth steps, sensor 1.73 m above a road with buildings, cars and
// poles) so results are reproducible without downloading KITTI. --no-reuse
// rebuilds the preprocessor and its buffers every frame, i.e. what a node
// that allocates per callback pays.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "perception_core/pointcloud_preprocessor.hpp"

namespace
{

using perception_core::PointXYZI;
using perception_core::PreprocessorParameters;

constexpr double kPi = 3.14159265358979323846;

PreprocessorParameters loadProfile(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  YAML::Node parameters;
  for (const auto & node : root) {
    if (node.second["ros__parameters"]) {
      parameters = node.second["ros__parameters"];
    }
  }
  if (!parameters) {
    throw std::runtime_error("No ros__parameters in " + path);
  }
  const auto get = [&parameters](const char * key, auto fallback) {
      return parameters[key] ? parameters[key].as<decltype(fallback)>() : fallback;
    };
  const auto vector = [&parameters](const char * key, const Eigen::Vector3d & fallback) {
      if (!parameters[key]) {
        return fallback;
      }
      const auto values = parameters[key].as<std::vector<double>>();
      return Eigen::Vector3d(values.at(0), values.at(1), values.at(2));
    };
  PreprocessorParameters p;
  p.filter.minimum = vector("crop.minimum", p.filter.minimum);
  p.filter.maximum = vector("crop.maximum", p.filter.maximum);
  p.filter.voxel_size = get("voxel_size", p.filter.voxel_size);
  p.filter.minimum_intensity = get("minimum_intensity", p.filter.minimum_intensity);
  p.remove_ground = get("ground.enabled", p.remove_ground);
  p.ground.distance_threshold = get("ground.distance_threshold", p.ground.distance_threshold);
  p.ground.max_iterations = get("ground.max_iterations", p.ground.max_iterations);
  p.ground.max_slope_deg = get("ground.max_slope_deg", p.ground.max_slope_deg);
  p.ground.min_ground_height = get("ground.min_height", p.ground.min_ground_height);
  p.ground.max_ground_height = get("ground.max_height", p.ground.max_ground_height);
  p.ground.min_inlier_ratio = get("ground.min_inlier_ratio", p.ground.min_inlier_ratio);
  p.cluster = get("clustering.enabled", p.cluster);
  p.clustering.tolerance = get("clustering.tolerance", p.clustering.tolerance);
  p.clustering.min_points = get("clustering.min_points", p.clustering.min_points);
  p.clustering.max_points = get("clustering.max_points", p.clustering.max_points);
  return p;
}

std::vector<PointXYZI> readKittiBin(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Cannot open " + path);
  }
  std::vector<PointXYZI> points;
  float record[4];
  while (file.read(reinterpret_cast<char *>(record), sizeof(record))) {
    points.push_back({Eigen::Vector3d(record[0], record[1], record[2]), record[3]});
  }
  return points;
}

struct Box
{
  Eigen::Vector3d minimum;
  Eigen::Vector3d maximum;
};

// Slab test; returns the entry distance or +inf.
double hitBox(const Eigen::Vector3d & origin, const Eigen::Vector3d & direction, const Box & box)
{
  double near = 0.0;
  double far = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(direction[axis]) < 1e-12) {
      if (origin[axis] < box.minimum[axis] || origin[axis] > box.maximum[axis]) {
        return std::numeric_limits<double>::infinity();
      }
      continue;
    }
    double t0 = (box.minimum[axis] - origin[axis]) / direction[axis];
    double t1 = (box.maximum[axis] - origin[axis]) / direction[axis];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    near = std::max(near, t0);
    far = std::min(far, t1);
    if (near > far) {
      return std::numeric_limits<double>::infinity();
    }
  }
  return near > 0.0 ? near : std::numeric_limits<double>::infinity();
}

// Sensor frame (like KITTI's velodyne frame): road at z = -1.73 with a 1%
// grade, building rows along both sides, parked cars and poles.
std::vector<PointXYZI> syntheticHdl64(std::uint32_t seed)
{
  std::vector<Box> boxes;
  boxes.push_back({{-60.0, 9.0, -1.8}, {60.0, 14.0, 8.0}});
  boxes.push_back({{-60.0, -14.0, -1.8}, {60.0, -10.0, 8.0}});
  for (int car = 0; car < 8; ++car) {
    const double x = -30.0 + 8.0 * car;
    const double y = (car % 2 == 0) ? 5.5 : -6.0;
    boxes.push_back({{x, y, -1.73}, {x + 4.2, y + 1.8, -0.28}});
  }
  for (int pole = 0; pole < 12; ++pole) {
    const double x = -40.0 + 7.0 * pole;
    boxes.push_back({{x, 8.0, -1.73}, {x + 0.25, 8.25, 3.0}});
  }
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, 0.02);
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  std::vector<PointXYZI> points;
  points.reserve(64 * 2083);
  for (int ring = 0; ring < 64; ++ring) {
    const double elevation = (-24.8 + ring * (26.8 / 63.0)) * kPi / 180.0;
    for (int step = 0; step < 2083; ++step) {
      const double azimuth = step * 2.0 * kPi / 2083.0;
      const Eigen::Vector3d direction(
        std::cos(elevation) * std::cos(azimuth), std::cos(elevation) * std::sin(azimuth),
        std::sin(elevation));
      double range = std::numeric_limits<double>::infinity();
      // Road plane z = -1.73 + 0.01 x.
      const double denominator = direction.z() - 0.01 * direction.x();
      if (denominator < -1e-9) {
        range = -1.73 / denominator;
      }
      for (const auto & box : boxes) {
        range = std::min(range, hitBox(origin, direction, box));
      }
      if (range > 120.0) {
        continue;  // no return
      }
      range += noise(rng);
      points.push_back({range * direction, 0.3});
    }
  }
  return points;
}

double percentile(std::vector<double> values, double fraction)
{
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(fraction * static_cast<double>(values.size() - 1));
  return values[index];
}

double mean(const std::vector<double> & values)
{
  double sum = 0.0;
  for (const double value : values) {
    sum += value;
  }
  return sum / static_cast<double>(values.size());
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(
      stderr,
      "usage: %s <profile.yaml> [--frames N] [--no-reuse] [kitti.bin ...]\n", argv[0]);
    return 2;
  }
  int frames = 50;
  bool reuse = true;
  std::vector<std::vector<PointXYZI>> clouds;
  for (int index = 2; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--frames" && index + 1 < argc) {
      frames = std::stoi(argv[++index]);
    } else if (argument == "--no-reuse") {
      reuse = false;
    } else {
      clouds.push_back(readKittiBin(argument));
    }
  }
  const bool synthetic = clouds.empty();
  if (synthetic) {
    for (std::uint32_t seed = 0; seed < 5; ++seed) {
      clouds.push_back(syntheticHdl64(seed));
    }
  }

  const PreprocessorParameters parameters = loadProfile(argv[1]);
  perception_core::PointCloudPreprocessor reused(parameters);
  std::vector<double> filter_ms, ground_ms, cluster_ms, total_ms;
  std::size_t points_in = 0;
  std::size_t obstacles = 0;
  std::size_t clusters = 0;
  for (int frame = -3; frame < frames; ++frame) {  // 3 warm-up frames
    const auto & cloud = clouds[static_cast<std::size_t>(frame + 3) % clouds.size()];
    perception_core::PreprocessorOutput output;
    if (reuse) {
      output = reused.process(cloud);
    } else {
      const auto start = std::chrono::steady_clock::now();
      perception_core::PointCloudPreprocessor fresh(parameters);
      output = fresh.process(cloud);
      output.timing.total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    }
    if (frame < 0) {
      continue;
    }
    filter_ms.push_back(output.timing.filter_ms);
    ground_ms.push_back(output.timing.ground_ms);
    cluster_ms.push_back(output.timing.cluster_ms);
    total_ms.push_back(output.timing.total_ms);
    points_in += cloud.size();
    obstacles += output.obstacles->size();
    clusters += output.clusters->size();
  }

  const double n = static_cast<double>(frames);
  std::printf(
    "source=%s frames=%d reuse=%s points/frame=%.0f obstacles/frame=%.0f clusters/frame=%.1f\n",
    synthetic ? "synthetic-hdl64" : "kitti-bin", frames, reuse ? "yes" : "no",
    static_cast<double>(points_in) / n, static_cast<double>(obstacles) / n,
    static_cast<double>(clusters) / n);
  std::printf("stage      mean_ms   p95_ms\n");
  std::printf("filter   %9.2f %8.2f\n", mean(filter_ms), percentile(filter_ms, 0.95));
  std::printf("ground   %9.2f %8.2f\n", mean(ground_ms), percentile(ground_ms, 0.95));
  std::printf("cluster  %9.2f %8.2f\n", mean(cluster_ms), percentile(cluster_ms, 0.95));
  std::printf("total    %9.2f %8.2f\n", mean(total_ms), percentile(total_ms, 0.95));
  std::printf(
    "throughput %.2f Mpoints/s\n",
    static_cast<double>(points_in) / (mean(total_ms) * n) / 1000.0);
  return 0;
}
