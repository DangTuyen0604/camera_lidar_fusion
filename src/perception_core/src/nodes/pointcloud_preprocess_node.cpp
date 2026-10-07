#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "perception_core/pointcloud_io.hpp"
#include "perception_core/pointcloud_preprocessor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace perception_core
{
namespace
{

Eigen::Vector3d vectorParameter(
  rclcpp::Node & node, const std::string & name,
  const Eigen::Vector3d & default_value)
{
  const auto values = node.declare_parameter<std::vector<double>>(
    name, {default_value.x(), default_value.y(), default_value.z()});
  if (values.size() != 3) {
    throw std::invalid_argument("Parameter " + name + " must have three values");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

Eigen::Isometry3d toIsometry(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & r = transform.transform.rotation;
  const auto & t = transform.transform.translation;
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized().toRotationMatrix();
  isometry.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  if (!isometry.matrix().allFinite()) {
    throw std::runtime_error("TF contains non-finite values");
  }
  return isometry;
}

}  // namespace

// Sensor-agnostic LiDAR preprocessing: PointCloud2 in any field layout ->
// target frame (TF) -> crop/voxel -> RANSAC ground removal -> Euclidean
// clusters. Everything sensor-specific is a parameter; see
// perception_core/config/*.yaml.
class PointCloudPreprocessNode : public rclcpp::Node
{
public:
  explicit PointCloudPreprocessNode(const rclcpp::NodeOptions & options)
  : Node("pointcloud_preprocess", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/lidar/points");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    // "sensor" publishes results back in the cloud's own frame. Consumers
    // that model the sensor from the message frame (STVL's LiDAR frustum
    // clearing) need that; processing still happens in target_frame.
    const auto output_frame = declare_parameter<std::string>("output_frame", "");
    if (!output_frame.empty() && output_frame != "sensor") {
      throw std::invalid_argument("output_frame must be \"\" or \"sensor\"");
    }
    output_in_sensor_frame_ = output_frame == "sensor";
    tf_timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("tf_timeout_ms", 50.0) / 1000.0);

    PreprocessorParameters parameters;
    parameters.filter.minimum = vectorParameter(*this, "crop.minimum", parameters.filter.minimum);
    parameters.filter.maximum = vectorParameter(*this, "crop.maximum", parameters.filter.maximum);
    parameters.filter.minimum_radius = declare_parameter<double>("crop.min_radius", 0.0);
    parameters.filter.voxel_size = declare_parameter<double>("voxel_size", 0.0);
    parameters.filter.minimum_intensity = declare_parameter<double>("minimum_intensity", 0.0);
    auto & ground = parameters.ground;
    parameters.remove_ground = declare_parameter<bool>("ground.enabled", true);
    ground.distance_threshold = declare_parameter<double>(
      "ground.distance_threshold", ground.distance_threshold);
    ground.max_iterations = declare_parameter<int>("ground.max_iterations", ground.max_iterations);
    ground.max_slope_deg = declare_parameter<double>("ground.max_slope_deg", ground.max_slope_deg);
    ground.min_ground_height = declare_parameter<double>(
      "ground.min_height", ground.min_ground_height);
    ground.max_ground_height = declare_parameter<double>(
      "ground.max_height", ground.max_ground_height);
    ground.min_inlier_ratio = declare_parameter<double>(
      "ground.min_inlier_ratio", ground.min_inlier_ratio);
    auto & clustering = parameters.clustering;
    parameters.cluster = declare_parameter<bool>("clustering.enabled", true);
    clustering.tolerance = declare_parameter<double>("clustering.tolerance", clustering.tolerance);
    clustering.min_points = static_cast<std::size_t>(
      declare_parameter<int>("clustering.min_points", static_cast<int>(clustering.min_points)));
    clustering.max_points = static_cast<std::size_t>(
      declare_parameter<int>("clustering.max_points", 100000));
    preprocessor_ = std::make_unique<PointCloudPreprocessor>(std::move(parameters));
    stats_period_ = declare_parameter<double>("stats_period_s", 10.0);

    obstacles_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("obstacles_topic", "/lidar/obstacles"),
      rclcpp::SensorDataQoS());
    ground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("ground_topic", "/lidar/ground"), rclcpp::SensorDataQoS());
    clusters_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      declare_parameter<std::string>("clusters_topic", "/lidar/clusters"), 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {onCloud(*cloud);});
    RCLCPP_INFO(
      get_logger(), "Preprocessing %s in frame '%s'", input_topic.c_str(),
      target_frame_.empty() ? "<sensor>" : target_frame_.c_str());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    const auto start = std::chrono::steady_clock::now();
    std_msgs::msg::Header header = cloud.header;
    bool transformed = false;
    try {
      readPointCloud(cloud, points_);
      if (!target_frame_.empty() && target_frame_ != cloud.header.frame_id) {
        to_target_ = toIsometry(
          tf_buffer_.lookupTransform(
            target_frame_, cloud.header.frame_id, rclcpp::Time(cloud.header.stamp),
            tf_timeout_));
        for (auto & point : points_) {
          point.position = to_target_ * point.position;
        }
        transformed = true;
        if (!output_in_sensor_frame_) {
          header.frame_id = target_frame_;
        }
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping cloud: %s", error.what());
      return;
    } catch (const std::invalid_argument & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Malformed PointCloud2: %s", error.what());
      return;
    }

    const PreprocessorOutput output = preprocessor_->process(points_);

    const bool back_to_sensor = transformed && output_in_sensor_frame_;
    const Eigen::Isometry3d to_sensor = back_to_sensor ? to_target_.inverse() :
      Eigen::Isometry3d::Identity();
    auto obstacles = std::make_unique<sensor_msgs::msg::PointCloud2>();
    writePointCloud(outputPoints(*output.obstacles, back_to_sensor, to_sensor), header, *obstacles);
    obstacles_publisher_->publish(std::move(obstacles));
    if (ground_publisher_->get_subscription_count() > 0) {
      auto ground = std::make_unique<sensor_msgs::msg::PointCloud2>();
      writePointCloud(outputPoints(*output.ground, back_to_sensor, to_sensor), header, *ground);
      ground_publisher_->publish(std::move(ground));
    }
    if (clusters_publisher_->get_subscription_count() > 0) {
      // Boxes stay axis-aligned in target_frame, where they are meaningful.
      std_msgs::msg::Header cluster_header = header;
      if (transformed) {
        cluster_header.frame_id = target_frame_;
      }
      publishClusters(*output.clusters, cluster_header);
    }
    accumulate(output, std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count());
  }

  const std::vector<PointXYZI> & outputPoints(
    const std::vector<PointXYZI> & points, bool back_to_sensor, const Eigen::Isometry3d & to_sensor)
  {
    if (!back_to_sensor) {
      return points;
    }
    output_points_ = points;
    for (auto & point : output_points_) {
      point.position = to_sensor * point.position;
    }
    return output_points_;
  }

  void publishClusters(const std::vector<Cluster> & clusters, const std_msgs::msg::Header & header)
  {
    auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers->markers.push_back(clear);
    int id = 0;
    for (const auto & cluster : clusters) {
      visualization_msgs::msg::Marker box;
      box.header = header;
      box.ns = "clusters";
      box.id = id++;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      const Eigen::Vector3d center = 0.5 * (cluster.minimum + cluster.maximum);
      const Eigen::Vector3d size = (cluster.maximum - cluster.minimum).cwiseMax(0.05);
      box.pose.position.x = center.x();
      box.pose.position.y = center.y();
      box.pose.position.z = center.z();
      box.pose.orientation.w = 1.0;
      box.scale.x = size.x();
      box.scale.y = size.y();
      box.scale.z = size.z();
      box.color.r = 1.0F;
      box.color.g = 0.6F;
      box.color.a = 0.5F;
      markers->markers.push_back(box);
    }
    clusters_publisher_->publish(std::move(markers));
  }

  // Periodic summary instead of per-frame logging: mean per-stage cost.
  void accumulate(const PreprocessorOutput & output, double callback_ms)
  {
    ++frames_;
    filter_ms_ += output.timing.filter_ms;
    ground_ms_ += output.timing.ground_ms;
    cluster_ms_ += output.timing.cluster_ms;
    callback_ms_ += callback_ms;
    points_in_ += points_.size();
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_stats_).count() < stats_period_) {
      return;
    }
    const double n = static_cast<double>(frames_);
    RCLCPP_INFO(
      get_logger(),
      "%zu frames, %.0f pts/frame: filter %.2f ms, ground %.2f ms, cluster %.2f ms, "
      "callback %.2f ms (mean)", frames_, static_cast<double>(points_in_) / n,
      filter_ms_ / n, ground_ms_ / n, cluster_ms_ / n, callback_ms_ / n);
    frames_ = 0;
    points_in_ = 0;
    filter_ms_ = ground_ms_ = cluster_ms_ = callback_ms_ = 0.0;
    last_stats_ = now;
  }

  std::string target_frame_;
  rclcpp::Duration tf_timeout_{0, 0};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<PointCloudPreprocessor> preprocessor_;
  bool output_in_sensor_frame_{false};
  Eigen::Isometry3d to_target_{Eigen::Isometry3d::Identity()};
  std::vector<PointXYZI> points_;
  std::vector<PointXYZI> output_points_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_publisher_;
  double stats_period_{10.0};
  std::chrono::steady_clock::time_point last_stats_{std::chrono::steady_clock::now()};
  std::size_t frames_{0};
  std::size_t points_in_{0};
  double filter_ms_{0.0};
  double ground_ms_{0.0};
  double cluster_ms_{0.0};
  double callback_ms_{0.0};
};

}  // namespace perception_core

RCLCPP_COMPONENTS_REGISTER_NODE(perception_core::PointCloudPreprocessNode)
