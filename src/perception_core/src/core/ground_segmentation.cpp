#include "perception_core/ground_segmentation.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <utility>

#include <Eigen/Eigenvalues>

namespace perception_core
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// Plane through three points, oriented so the normal points up.
bool planeThrough(
  const Eigen::Vector3d & a, const Eigen::Vector3d & b, const Eigen::Vector3d & c,
  GroundPlane & plane)
{
  Eigen::Vector3d normal = (b - a).cross(c - a);
  const double norm = normal.norm();
  if (norm < 1.0e-9) {
    return false;  // collinear sample
  }
  normal /= norm;
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  plane.normal = normal;
  plane.offset = -normal.dot(a);
  return true;
}

}  // namespace

RansacGroundSegmenter::RansacGroundSegmenter(GroundSegmentationParameters parameters)
: parameters_(std::move(parameters))
{
  if (!(parameters_.distance_threshold > 0.0) || parameters_.max_iterations < 1 ||
    !(parameters_.max_slope_deg >= 0.0 && parameters_.max_slope_deg < 90.0) ||
    !(parameters_.max_ground_height >= parameters_.min_ground_height) ||
    !(parameters_.min_inlier_ratio >= 0.0 && parameters_.min_inlier_ratio <= 1.0))
  {
    throw std::invalid_argument("Invalid ground segmentation parameters");
  }
  min_normal_z_ = std::cos(parameters_.max_slope_deg * kPi / 180.0);
}

bool RansacGroundSegmenter::acceptable(const GroundPlane & plane) const
{
  if (plane.normal.z() < min_normal_z_) {
    return false;
  }
  // Height of the plane straight below/above the frame origin.
  const double height = -plane.offset / plane.normal.z();
  return height >= parameters_.min_ground_height && height <= parameters_.max_ground_height;
}

GroundPlane RansacGroundSegmenter::segment(
  const std::vector<PointXYZI> & points,
  std::vector<PointXYZI> & ground,
  std::vector<PointXYZI> & obstacles)
{
  ground.clear();
  obstacles.clear();
  const double threshold = parameters_.distance_threshold;

  // A sloped floor rises with distance; widen the candidate band accordingly
  // instead of sampling the whole cloud (walls and boxes would win).
  candidates_.clear();
  const double slope = std::tan(parameters_.max_slope_deg * kPi / 180.0);
  for (std::size_t index = 0; index < points.size(); ++index) {
    const Eigen::Vector3d & p = points[index].position;
    const double reach = slope * std::hypot(p.x(), p.y());
    if (p.z() >= parameters_.min_ground_height - reach - threshold &&
      p.z() <= parameters_.max_ground_height + reach + threshold)
    {
      candidates_.push_back(index);
    }
  }

  GroundPlane best;
  if (candidates_.size() >= 3) {
    std::mt19937 rng(parameters_.seed);
    std::uniform_int_distribution<std::size_t> pick(0, candidates_.size() - 1);
    for (int iteration = 0; iteration < parameters_.max_iterations; ++iteration) {
      const std::size_t i = candidates_[pick(rng)];
      const std::size_t j = candidates_[pick(rng)];
      const std::size_t k = candidates_[pick(rng)];
      GroundPlane hypothesis;
      if (i == j || j == k || i == k ||
        !planeThrough(points[i].position, points[j].position, points[k].position, hypothesis) ||
        !acceptable(hypothesis))
      {
        continue;
      }
      std::size_t count = 0;
      for (const std::size_t index : candidates_) {
        if (std::abs(hypothesis.signedDistance(points[index].position)) < threshold) {
          ++count;
        }
      }
      if (count > best.inliers) {
        best = hypothesis;
        best.inliers = count;
        best.valid = true;
      }
    }
  }

  const auto required = static_cast<std::size_t>(
    std::ceil(parameters_.min_inlier_ratio * static_cast<double>(candidates_.size())));
  if (best.valid && best.inliers >= std::max<std::size_t>(3, required)) {
    // Least-squares refit: normal = eigenvector of the smallest eigenvalue of
    // the inlier covariance. Three random points tilt the plane; the refit
    // averages that out.
    inliers_.clear();
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const std::size_t index : candidates_) {
      if (std::abs(best.signedDistance(points[index].position)) < threshold) {
        inliers_.push_back(index);
        centroid += points[index].position;
      }
    }
    centroid /= static_cast<double>(inliers_.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const std::size_t index : inliers_) {
      const Eigen::Vector3d d = points[index].position - centroid;
      covariance += d * d.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    GroundPlane refined = best;
    refined.normal = solver.eigenvectors().col(0);
    if (refined.normal.z() < 0.0) {
      refined.normal = -refined.normal;
    }
    refined.offset = -refined.normal.dot(centroid);
    if (solver.info() == Eigen::Success && acceptable(refined)) {
      best = refined;
    }
  } else {
    best = GroundPlane{};
  }

  ground.reserve(best.valid ? best.inliers : 0);
  obstacles.reserve(points.size());
  for (const auto & point : points) {
    if (best.valid && std::abs(best.signedDistance(point.position)) < threshold) {
      ground.push_back(point);
    } else {
      obstacles.push_back(point);
    }
  }
  best.inliers = ground.size();
  return best;
}

const GroundSegmentationParameters & RansacGroundSegmenter::parameters() const noexcept
{
  return parameters_;
}

}  // namespace perception_core
