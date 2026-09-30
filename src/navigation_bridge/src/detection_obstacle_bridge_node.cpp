#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "fusion_interfaces/msg/fused_detection_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "navigation_bridge/detection_obstacle_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace navigation_bridge
{

class DetectionObstacleBridgeNode : public rclcpp::Node
{
public:
  DetectionObstacleBridgeNode()
  : Node("detection_obstacle_bridge"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/fusion/detections_3d");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/navigation/detection_obstacles");
    const auto clearing_topic = declare_parameter<std::string>(
      "clearing_topic", "/navigation/detection_obstacles/clearing");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    config_.minimum_confidence = declare_parameter<double>("minimum_confidence", 0.35);
    config_.minimum_range = declare_parameter<double>("minimum_range", 0.20);
    config_.maximum_range = declare_parameter<double>("maximum_range", 30.0);
    config_.footprint_resolution = declare_parameter<double>("footprint_resolution", 0.10);
    stale_timeout_ = declare_parameter<double>("stale_timeout", 0.50);
    future_tolerance_ = declare_parameter<double>("future_tolerance", 0.10);
    obstacle_timeout_ = declare_parameter<double>("obstacle_timeout", 0.75);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.10);
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    clearing_history_ = declare_parameter<double>("clearing_history", 1.0);
    if (target_frame_.empty() || fixed_frame_.empty() || clearing_history_ < 0.0 ||
      stale_timeout_ <= 0.0 || future_tolerance_ < 0.0 ||
      obstacle_timeout_ <= 0.0 || tf_timeout_ < 0.0)
    {
      throw std::invalid_argument("Invalid frame or timeout parameter");
    }

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS());
    clearing_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      clearing_topic, rclcpp::SensorDataQoS());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    subscription_ = create_subscription<fusion_interfaces::msg::FusedDetectionArray>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&DetectionObstacleBridgeNode::callback, this, std::placeholders::_1));
    timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&DetectionObstacleBridgeNode::onTimer, this));
    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&DetectionObstacleBridgeNode::publishDiagnostics, this));
  }

private:
  static diagnostic_msgs::msg::KeyValue keyValue(
    const std::string & key, std::uint64_t value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(value);
    return item;
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<ObstaclePoint> & points, const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = target_frame_;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & point : points) {
      *x = point.x;
      *y = point.y;
      *z = point.z;
      ++x;
      ++y;
      ++z;
    }
    return cloud;
  }

  void reject(std::uint64_t & counter, const std::string & reason)
  {
    ++counter;
    last_status_ = reason;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reason.c_str());
  }

  void callback(const fusion_interfaces::msg::FusedDetectionArray::ConstSharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      reject(rejected_empty_frame_, "Rejected detections with empty frame_id");
      return;
    }
    const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      reject(rejected_zero_stamp_, "Rejected detections with zero timestamp");
      return;
    }
    const auto age = (now() - stamp).seconds();
    if (age > stale_timeout_) {
      reject(rejected_stale_, "Rejected stale detections");
      return;
    }
    if (age < -future_tolerance_) {
      reject(rejected_future_, "Rejected future detections");
      return;
    }

    RejectionCounters batch;
    auto candidates = DetectionObstacleBridge::extractCandidates(*message, config_, &batch);
    counters_.accepted += batch.accepted;
    counters_.invalid += batch.invalid;
    counters_.confidence += batch.confidence;
    counters_.range += batch.range;
    counters_.non_finite += batch.non_finite;

    // An empty (or fully rejected) batch is not a fresh obstacle observation.
    // Do not move last_observation_ forward: the timeout path must still clear
    // points published by the last valid batch.
    if (candidates.empty()) {
      ++rejected_empty_batch_;
      last_status_ = "No valid detections in batch";
      return;
    }

    std::vector<ObstaclePoint> transformed;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, message->header.frame_id, stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
      for (const auto & candidate : candidates) {
        geometry_msgs::msg::PointStamped input;
        geometry_msgs::msg::PointStamped output;
        input.header = message->header;
        input.point.x = candidate.position.x;
        input.point.y = candidate.position.y;
        input.point.z = candidate.position.z;
        tf2::doTransform(input, output, transform);
        if (!std::isfinite(output.point.x) || !std::isfinite(output.point.y) ||
          !std::isfinite(output.point.z))
        {
          ++rejected_transformed_non_finite_;
          last_status_ = "TF produced a non-finite obstacle point";
          return;
        }
        const ObstaclePoint center{
          static_cast<float>(output.point.x), static_cast<float>(output.point.y),
          static_cast<float>(output.point.z)};
        auto footprint = DetectionObstacleBridge::expandFootprint(
          center, candidate.class_name, config_.footprint_resolution);
        transformed.insert(transformed.end(), footprint.begin(), footprint.end());
      }
    } catch (const tf2::TransformException & error) {
      ++rejected_tf_;
      last_status_ = std::string("Missing TF: ") + error.what();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot transform detections: %s", error.what());
      return;
    }

    // A moving worker must not leave a trail of stale marks from A to B.
    // Clear where the recent batches marked before marking the new batch;
    // the costmap applies clearing before marking in the same update.
    publishClearing(stamp);
    publisher_->publish(makeCloud(transformed, stamp));
    history_.push_back({stamp, std::move(transformed)});
    last_observation_ = now();
    has_observation_ = true;
    clearing_sent_ = false;
    last_status_ = "OK";
  }

  // Raytrace slightly beyond every point marked during the last
  // clearing_history seconds so the costmap frees those cells. The costmap
  // keeps only the newest clearing cloud, and a 2 Hz global costmap skips
  // most batches, so clearing only the previous batch leaves a trail of every
  // skipped batch's marks. Each batch is in target_frame at its own capture
  // time; move it through fixed_frame into target_frame at `stamp` so the
  // rays land on the marks while the robot drives.
  void publishClearing(const rclcpp::Time & stamp)
  {
    while (!history_.empty() &&
      (stamp - history_.front().stamp).seconds() > clearing_history_)
    {
      history_.pop_front();
    }
    std::vector<ObstaclePoint> rays;
    for (const auto & batch : history_) {
      tf2::Transform motion;
      motion.setIdentity();
      if (batch.stamp != stamp) {
        try {
          tf2::fromMsg(
            tf_buffer_.lookupTransform(
              target_frame_, stamp, target_frame_, batch.stamp, fixed_frame_,
              rclcpp::Duration::from_seconds(tf_timeout_)).transform, motion);
        } catch (const tf2::TransformException & error) {
          // Without odometry, clear where the points were in the robot frame.
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Clearing without robot motion compensation: %s", error.what());
        }
      }
      for (const auto & p : batch.points) {
        const auto q = motion * tf2::Vector3(p.x, p.y, p.z);
        rays.push_back({
            static_cast<float>(q.x() * 1.05), static_cast<float>(q.y() * 1.05),
            static_cast<float>(q.z())});
      }
    }
    if (rays.empty()) {
      return;
    }
    clearing_publisher_->publish(makeCloud(rays, stamp));
    ++clearing_publications_;
  }

  void onTimer()
  {
    // Collision Monitor treats a source that has never published as failed
    // and stops the robot.  An empty cloud is the correct observation while
    // the perception pipeline is healthy but has not detected an obstacle.
    if (!has_observation_) {
      publisher_->publish(makeCloud({}, now()));
      return;
    }
    if ((now() - last_observation_).seconds() <= obstacle_timeout_) {
      return;
    }
    if (!clearing_sent_) {
      if (!history_.empty()) {
        publishClearing(history_.back().stamp);
      }
      history_.clear();
      clearing_sent_ = true;
      last_status_ = "Obstacle timeout: clearing published";
    }
    // Collision Monitor treats a silent PointCloud source as failed. Keep a
    // fresh empty observation flowing after expiry while publishing the
    // costmap clearing rays only once.
    publisher_->publish(makeCloud({}, now()));
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "navigation_bridge/detection_obstacle_bridge";
    status.hardware_id = "fusion_obstacle_bridge";
    status.level = rejected_tf_ > 0 && counters_.accepted == 0 ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = last_status_;
    status.values = {
      keyValue("accepted", counters_.accepted), keyValue("rejected_invalid", counters_.invalid),
      keyValue("rejected_confidence", counters_.confidence),
      keyValue("rejected_range", counters_.range),
      keyValue("rejected_non_finite", counters_.non_finite),
      keyValue("rejected_empty_batch", rejected_empty_batch_),
      keyValue("rejected_empty_frame", rejected_empty_frame_),
      keyValue("rejected_zero_stamp", rejected_zero_stamp_),
      keyValue("rejected_stale", rejected_stale_), keyValue("rejected_future", rejected_future_),
      keyValue("rejected_missing_tf", rejected_tf_),
      keyValue("rejected_transformed_non_finite", rejected_transformed_non_finite_),
      keyValue("clearing_publications", clearing_publications_)};
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  BridgeConfig config_;
  std::string target_frame_;
  double stale_timeout_{0.5};
  double future_tolerance_{0.1};
  double obstacle_timeout_{0.75};
  double tf_timeout_{0.1};
  std::string fixed_frame_;
  double clearing_history_{1.0};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  RejectionCounters counters_;
  std::uint64_t rejected_empty_frame_{0};
  std::uint64_t rejected_zero_stamp_{0};
  std::uint64_t rejected_stale_{0};
  std::uint64_t rejected_future_{0};
  std::uint64_t rejected_tf_{0};
  std::uint64_t rejected_empty_batch_{0};
  std::uint64_t rejected_transformed_non_finite_{0};
  std::uint64_t clearing_publications_{0};
  std::string last_status_{"Waiting for detections"};
  rclcpp::Time last_observation_{0, 0, RCL_ROS_TIME};
  bool has_observation_{false};
  bool clearing_sent_{false};
  struct Batch
  {
    rclcpp::Time stamp;
    std::vector<ObstaclePoint> points;
  };
  std::deque<Batch> history_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearing_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<fusion_interfaces::msg::FusedDetectionArray>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace navigation_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation_bridge::DetectionObstacleBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
