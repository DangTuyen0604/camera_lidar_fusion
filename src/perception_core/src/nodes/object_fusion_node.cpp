#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

#include "fusion_interfaces/msg/detection2_d_array.hpp"
#include "fusion_interfaces/msg/fused_detection.hpp"
#include "fusion_interfaces/msg/fused_detection_array.hpp"
#include "perception_core/lidar_projector.hpp"
#include "perception_core/object_depth_estimator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace perception_core
{
namespace
{

std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

template<typename MessageT>
typename MessageT::ConstSharedPtr nearestMessage(
  const std::deque<typename MessageT::ConstSharedPtr> & messages,
  std::int64_t target_ns,
  double * difference_ms)
{
  typename MessageT::ConstSharedPtr nearest;
  std::int64_t nearest_difference = std::numeric_limits<std::int64_t>::max();
  for (const auto & message : messages) {
    const std::int64_t difference = std::llabs(
      stampNanoseconds(message->header.stamp) - target_ns);
    if (difference < nearest_difference) {
      nearest = message;
      nearest_difference = difference;
    }
  }
  if (difference_ms != nullptr) {
    *difference_ms = nearest ? static_cast<double>(nearest_difference) / 1.0e6 :
      std::numeric_limits<double>::infinity();
  }
  return nearest;
}

template<typename MessageT>
void appendBounded(
  std::deque<typename MessageT::ConstSharedPtr> & messages,
  const typename MessageT::ConstSharedPtr & message,
  std::size_t maximum_size)
{
  messages.push_back(message);
  while (messages.size() > maximum_size) {
    messages.pop_front();
  }
}

const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name)
{
  const auto iterator = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {return field.name == name;});
  return iterator == cloud.fields.end() ? nullptr : &(*iterator);
}

float readFloat32(const std::uint8_t * data, bool is_bigendian)
{
  std::uint8_t bytes[sizeof(float)];
  std::memcpy(bytes, data, sizeof(float));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  if (is_bigendian) {
    std::reverse(bytes, bytes + sizeof(float));
  }
#else
  if (!is_bigendian) {
    std::reverse(bytes, bytes + sizeof(float));
  }
#endif
  float value;
  std::memcpy(&value, bytes, sizeof(float));
  return value;
}

std::vector<PointXYZI> readPointCloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  const auto * x_field = findField(cloud, "x");
  const auto * y_field = findField(cloud, "y");
  const auto * z_field = findField(cloud, "z");
  const auto * intensity_field = findField(cloud, "intensity");
  const std::uint8_t float32 = sensor_msgs::msg::PointField::FLOAT32;
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr ||
    x_field->datatype != float32 || y_field->datatype != float32 ||
    z_field->datatype != float32 || cloud.point_step == 0)
  {
    throw std::runtime_error("PointCloud2 must contain FLOAT32 x, y and z fields");
  }
  const auto field_fits = [&cloud](const sensor_msgs::msg::PointField * field) {
      return field != nullptr && field->offset + sizeof(float) <= cloud.point_step;
    };
  if (!field_fits(x_field) || !field_fits(y_field) || !field_fits(z_field) ||
    (intensity_field != nullptr &&
    (intensity_field->datatype != float32 || !field_fits(intensity_field))))
  {
    throw std::runtime_error("PointCloud2 field offsets exceed point_step");
  }

  const std::size_t count = static_cast<std::size_t>(cloud.width) * cloud.height;
  if (cloud.data.size() < count * cloud.point_step) {
    throw std::runtime_error("PointCloud2 data buffer is smaller than its declared layout");
  }
  std::vector<PointXYZI> points;
  points.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const std::uint8_t * data = cloud.data.data() + index * cloud.point_step;
    PointXYZI point;
    point.position = Eigen::Vector3d(
      readFloat32(data + x_field->offset, cloud.is_bigendian),
      readFloat32(data + y_field->offset, cloud.is_bigendian),
      readFloat32(data + z_field->offset, cloud.is_bigendian));
    point.intensity = intensity_field == nullptr ? 0.0 :
      readFloat32(data + intensity_field->offset, cloud.is_bigendian);
    points.push_back(point);
  }
  return points;
}

Eigen::Matrix4d transformMatrix(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & rotation = transform.transform.rotation;
  Eigen::Quaterniond quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-12) {
    throw std::runtime_error("TF contains an invalid quaternion");
  }
  quaternion.normalize();
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.topLeftCorner<3, 3>() = quaternion.toRotationMatrix();
  matrix(0, 3) = transform.transform.translation.x;
  matrix(1, 3) = transform.transform.translation.y;
  matrix(2, 3) = transform.transform.translation.z;
  if (!matrix.allFinite()) {
    throw std::runtime_error("TF contains non-finite values");
  }
  return matrix;
}

}  // namespace

class ObjectFusionNode : public rclcpp::Node
{
public:
  using DetectionArray = fusion_interfaces::msg::Detection2DArray;
  using FusedArray = fusion_interfaces::msg::FusedDetectionArray;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using PointCloud = sensor_msgs::msg::PointCloud2;

  ObjectFusionNode()
  : Node("object_fusion_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const std::string detections_topic = declare_parameter<std::string>(
      "detections_topic", "/detections_2d");
    const std::string image_topic = declare_parameter<std::string>(
      "image_topic", "/fusion/synced/image");
    const std::string pointcloud_topic = declare_parameter<std::string>(
      "pointcloud_topic", "/fusion/synced/points");
    const std::string camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/fusion/synced/camera_info");
    const std::string fused_topic = declare_parameter<std::string>(
      "fused_detections_topic", "/fusion/detections_3d");
    const std::string annotated_topic = declare_parameter<std::string>(
      "annotated_image_topic", "/fusion/annotated_image");
    const std::string marker_topic = declare_parameter<std::string>(
      "marker_topic", "/fusion/object_markers");
    camera_frame_id_ = declare_parameter<std::string>(
      "camera_frame_id", "camera_optical_frame");
    lidar_frame_id_ = declare_parameter<std::string>("lidar_frame_id", "velodyne");
    sync_tolerance_ms_ = declare_parameter<double>("sync_tolerance_ms", 50.0);
    tf_timeout_ms_ = declare_parameter<double>("tf_timeout_ms", 20.0);
    marker_lifetime_sec_ = declare_parameter<double>("marker_lifetime_sec", 0.35);
    const int cache_size = declare_parameter<int>("cache_size", 30);

    DepthEstimatorParameters parameters;
    const int min_lidar_points = declare_parameter<int>("min_lidar_points", 3);
    parameters.min_lidar_points = min_lidar_points > 0 ?
      static_cast<std::size_t>(min_lidar_points) : 0U;
    parameters.bbox_shrink_ratio = declare_parameter<double>("bbox_shrink_ratio", 0.7);
    parameters.min_depth = declare_parameter<double>("min_depth", 1.0);
    parameters.max_depth = declare_parameter<double>("max_depth", 80.0);
    parameters.depth_cluster_gap = declare_parameter<double>("depth_cluster_gap", 1.0);
    parameters.outlier_threshold = declare_parameter<double>("outlier_threshold", 2.0);
    parameters.minimum_outlier_window = declare_parameter<double>(
      "minimum_outlier_window", 0.25);
    estimator_ = std::make_unique<ObjectDepthEstimator>(parameters);

    if (!std::isfinite(sync_tolerance_ms_) || sync_tolerance_ms_ < 0.0 ||
      !std::isfinite(tf_timeout_ms_) || tf_timeout_ms_ < 0.0 ||
      !std::isfinite(marker_lifetime_sec_) || marker_lifetime_sec_ <= 0.0 ||
      cache_size <= 0)
    {
      throw std::invalid_argument("Invalid object fusion runtime parameters");
    }
    cache_size_ = static_cast<std::size_t>(cache_size);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    fused_publisher_ = create_publisher<FusedArray>(fused_topic, 10);
    annotated_image_publisher_ = create_publisher<Image>(annotated_topic, sensor_qos);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);
    image_subscription_ = create_subscription<Image>(
      image_topic, sensor_qos,
      [this](const Image::ConstSharedPtr message) {
        appendBounded<Image>(images_, message, cache_size_);
      });
    camera_info_subscription_ = create_subscription<CameraInfo>(
      camera_info_topic, sensor_qos,
      [this](const CameraInfo::ConstSharedPtr message) {
        appendBounded<CameraInfo>(camera_infos_, message, cache_size_);
      });
    pointcloud_subscription_ = create_subscription<PointCloud>(
      pointcloud_topic, sensor_qos,
      [this](const PointCloud::ConstSharedPtr message) {
        appendBounded<PointCloud>(pointclouds_, message, cache_size_);
      });
    detection_subscription_ = create_subscription<DetectionArray>(
      detections_topic, 10,
      std::bind(&ObjectFusionNode::detectionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Object fusion ready: tolerance=%.1f ms, min_lidar_points=%zu",
      sync_tolerance_ms_, parameters.min_lidar_points);
  }

private:
  void detectionCallback(const DetectionArray::ConstSharedPtr detection_array)
  {
    const std::int64_t detection_ns = stampNanoseconds(detection_array->header.stamp);
    double cloud_difference_ms;
    double info_difference_ms;
    double image_difference_ms;
    const auto cloud = nearestMessage<PointCloud>(
      pointclouds_, detection_ns, &cloud_difference_ms);
    const auto camera_info = nearestMessage<CameraInfo>(
      camera_infos_, detection_ns, &info_difference_ms);
    const auto image = nearestMessage<Image>(images_, detection_ns, &image_difference_ms);

    FusedArray output;
    output.header = detection_array->header;
    output.header.frame_id = camera_frame_id_;

    const bool synchronized = cloud && camera_info &&
      cloud_difference_ms <= sync_tolerance_ms_ &&
      info_difference_ms <= sync_tolerance_ms_;
    const bool frames_valid = synchronized &&
      cloud->header.frame_id == lidar_frame_id_ &&
      camera_info->header.frame_id == camera_frame_id_ &&
      detection_array->header.frame_id == camera_frame_id_;

    std::vector<DepthEstimate> estimates(detection_array->detections.size());
    if (frames_valid) {
      try {
        estimates = estimateDepths(*detection_array, *cloud, *camera_info);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN(get_logger(), "TF lookup failed at detection timestamp: %s", error.what());
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "Fusion rejected input: %s", error.what());
      }
    } else if (cloud && cloud->header.frame_id != lidar_frame_id_) {
      RCLCPP_WARN(
        get_logger(), "Rejected cloud frame: expected=%s actual=%s",
        lidar_frame_id_.c_str(), cloud->header.frame_id.c_str());
    }

    output.detections.reserve(detection_array->detections.size());
    for (std::size_t index = 0; index < detection_array->detections.size(); ++index) {
      fusion_interfaces::msg::FusedDetection fused;
      fused.header = output.header;
      fused.detection = detection_array->detections[index];
      fused.valid = estimates[index].valid;
      if (fused.valid) {
        fused.position.x = estimates[index].position.x();
        fused.position.y = estimates[index].position.y();
        fused.position.z = estimates[index].position.z();
        fused.depth = static_cast<float>(estimates[index].depth);
        fused.lidar_point_count = static_cast<std::uint32_t>(
          estimates[index].lidar_point_count);
      }
      output.detections.push_back(fused);
    }
    fused_publisher_->publish(output);
    publishMarkers(output);

    if (image && image_difference_ms <= sync_tolerance_ms_) {
      publishAnnotatedImage(*image, output);
    }
  }

  std::vector<DepthEstimate> estimateDepths(
    const DetectionArray & detections,
    const PointCloud & cloud,
    const CameraInfo & camera_info)
  {
    if (camera_info.width == 0 || camera_info.height == 0) {
      throw std::runtime_error("CameraInfo image dimensions are invalid");
    }
    CalibrationData calibration;
    calibration.image_width = static_cast<int>(camera_info.width);
    calibration.image_height = static_cast<int>(camera_info.height);
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 4; ++column) {
        calibration.projection_matrix(row, column) = camera_info.p[row * 4 + column];
      }
    }
    const auto transform = tf_buffer_.lookupTransform(
      camera_frame_id_, cloud.header.frame_id, rclcpp::Time(detections.header.stamp),
      rclcpp::Duration::from_seconds(tf_timeout_ms_ / 1000.0));
    calibration.lidar_to_camera = transformMatrix(transform);
    calibration.lidar_frame = cloud.header.frame_id;
    calibration.camera_frame = camera_frame_id_;

    const LidarProjector projector(std::move(calibration));
    const auto projected = projector.project(
      readPointCloud(cloud), static_cast<int>(camera_info.width),
      static_cast<int>(camera_info.height), estimator_->parameters().min_depth,
      estimator_->parameters().max_depth);

    std::vector<DepthEstimate> estimates;
    estimates.reserve(detections.detections.size());
    for (const auto & detection : detections.detections) {
      const BoundingBox2D bounding_box{
        detection.x_min, detection.y_min, detection.x_max, detection.y_max};
      estimates.push_back(estimator_->estimate(bounding_box, projected));
    }
    return estimates;
  }

  void publishAnnotatedImage(const Image & image, const FusedArray & detections)
  {
    try {
      cv_bridge::CvImagePtr converted = cv_bridge::toCvCopy(image, "bgr8");
      if (converted->image.empty()) {
        return;
      }
      for (const auto & fused : detections.detections) {
        const auto & detection = fused.detection;
        const cv::Scalar color = fused.valid ? cv::Scalar(40, 220, 40) :
          cv::Scalar(40, 40, 220);
        const int x_min = std::clamp(
          static_cast<int>(std::lround(detection.x_min)), 0, converted->image.cols - 1);
        const int y_min = std::clamp(
          static_cast<int>(std::lround(detection.y_min)), 0, converted->image.rows - 1);
        const int x_max = std::clamp(
          static_cast<int>(std::lround(detection.x_max)), 0, converted->image.cols - 1);
        const int y_max = std::clamp(
          static_cast<int>(std::lround(detection.y_max)), 0, converted->image.rows - 1);
        cv::rectangle(converted->image, cv::Point(x_min, y_min), cv::Point(x_max, y_max), color, 2);

        std::ostringstream label;
        label << detection.class_name << " " << std::fixed << std::setprecision(2) <<
          detection.confidence;
        if (fused.valid) {
          label << " | depth=" << std::setprecision(1) << fused.depth <<
            " m | points=" << fused.lidar_point_count;
        } else {
          label << " | depth=invalid";
        }
        cv::putText(
          converted->image, label.str(), cv::Point(x_min, std::max(18, y_min - 6)),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
      }
      converted->header = detections.header;
      annotated_image_publisher_->publish(*converted->toImageMsg());
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_WARN(get_logger(), "Cannot draw annotated image: %s", error.what());
    }
  }

  void publishMarkers(const FusedArray & detections)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int marker_id = 0;
    for (std::size_t index = 0; index < detections.detections.size(); ++index) {
      const auto & fused = detections.detections[index];
      visualization_msgs::msg::Marker marker;
      marker.header = detections.header;
      marker.ns = "fused_objects";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.pose.position = fused.position;
      if (!fused.valid) {
        marker.pose.position.z = 1.0 + static_cast<double>(index) * 0.25;
      }
      marker.scale.x = fused.valid ? 0.6 : 0.2;
      marker.scale.y = fused.valid ? 0.6 : 0.2;
      marker.scale.z = fused.valid ? 0.6 : 0.2;
      marker.color.a = 0.9F;
      marker.color.r = fused.valid ? 0.1F : 0.8F;
      marker.color.g = fused.valid ? 0.9F : 0.1F;
      marker.color.b = 0.1F;
      marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
      markers.markers.push_back(marker);

      visualization_msgs::msg::Marker text = marker;
      text.id = marker_id++;
      text.ns = "fused_labels";
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position.y -= 0.4;
      text.scale.x = 0.0;
      text.scale.y = 0.0;
      text.scale.z = 0.35;
      std::ostringstream label;
      label << fused.detection.class_name << " " << std::fixed << std::setprecision(2) <<
        fused.detection.confidence;
      if (fused.valid) {
        label << " | " << std::setprecision(1) << fused.depth << " m | " <<
          fused.lidar_point_count << " pts";
      } else {
        label << " | invalid";
      }
      text.text = label.str();
      markers.markers.push_back(text);
    }
    marker_publisher_->publish(markers);
  }

  std::string camera_frame_id_;
  std::string lidar_frame_id_;
  double sync_tolerance_ms_{50.0};
  double tf_timeout_ms_{20.0};
  double marker_lifetime_sec_{0.35};
  std::size_t cache_size_{30};
  std::unique_ptr<ObjectDepthEstimator> estimator_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::deque<Image::ConstSharedPtr> images_;
  std::deque<CameraInfo::ConstSharedPtr> camera_infos_;
  std::deque<PointCloud::ConstSharedPtr> pointclouds_;
  rclcpp::Publisher<FusedArray>::SharedPtr fused_publisher_;
  rclcpp::Publisher<Image>::SharedPtr annotated_image_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<DetectionArray>::SharedPtr detection_subscription_;
  rclcpp::Subscription<Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<PointCloud>::SharedPtr pointcloud_subscription_;
};

}  // namespace perception_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_core::ObjectFusionNode>());
  rclcpp::shutdown();
  return 0;
}
