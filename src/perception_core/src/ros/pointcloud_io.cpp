#include "perception_core/pointcloud_io.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include "sensor_msgs/msg/point_field.hpp"

namespace perception_core
{
namespace
{

using sensor_msgs::msg::PointField;

std::size_t datatypeSize(std::uint8_t datatype)
{
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      return 1;
    case PointField::INT16:
    case PointField::UINT16:
      return 2;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      return 4;
    case PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

template<typename T>
T loadValue(const std::uint8_t * data, bool swap)
{
  std::uint8_t bytes[sizeof(T)];
  std::memcpy(bytes, data, sizeof(T));
  if (swap) {
    std::reverse(bytes, bytes + sizeof(T));
  }
  T value;
  std::memcpy(&value, bytes, sizeof(T));
  return value;
}

// Resolved once per message so the per-point loop only dispatches on a
// pre-validated datatype.
struct FieldReader
{
  std::uint32_t offset{0};
  std::uint8_t datatype{0};

  double read(const std::uint8_t * point, bool swap) const
  {
    const std::uint8_t * data = point + offset;
    switch (datatype) {
      case PointField::INT8:
        return loadValue<std::int8_t>(data, false);
      case PointField::UINT8:
        return loadValue<std::uint8_t>(data, false);
      case PointField::INT16:
        return loadValue<std::int16_t>(data, swap);
      case PointField::UINT16:
        return loadValue<std::uint16_t>(data, swap);
      case PointField::INT32:
        return loadValue<std::int32_t>(data, swap);
      case PointField::UINT32:
        return loadValue<std::uint32_t>(data, swap);
      case PointField::FLOAT32:
        return loadValue<float>(data, swap);
      default:
        return loadValue<double>(data, swap);
    }
  }
};

bool fieldReader(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name,
  FieldReader & reader)
{
  const auto field = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const PointField & candidate) {return candidate.name == name;});
  if (field == cloud.fields.end()) {
    return false;
  }
  const std::size_t size = datatypeSize(field->datatype);
  if (size == 0) {
    throw std::invalid_argument(
            "PointCloud2 field '" + name + "' has unsupported datatype " +
            std::to_string(field->datatype));
  }
  if (field->count < 1 || field->offset + size > cloud.point_step) {
    throw std::invalid_argument(
            "PointCloud2 field '" + name + "' does not fit in point_step");
  }
  reader.offset = field->offset;
  reader.datatype = field->datatype;
  return true;
}

bool hostIsBigEndian()
{
  const std::uint16_t probe = 1;
  std::uint8_t first;
  std::memcpy(&first, &probe, 1);
  return first == 0;
}

void appendFloat32Field(
  sensor_msgs::msg::PointCloud2 & cloud, const char * name, std::uint32_t offset)
{
  PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = PointField::FLOAT32;
  field.count = 1;
  cloud.fields.push_back(field);
}

}  // namespace

PointCloudReadStats readPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  std::vector<PointXYZI> & points)
{
  points.clear();
  FieldReader x;
  FieldReader y;
  FieldReader z;
  if (!fieldReader(cloud, "x", x) || !fieldReader(cloud, "y", y) ||
    !fieldReader(cloud, "z", z))
  {
    throw std::invalid_argument("PointCloud2 must contain x, y and z fields");
  }
  FieldReader intensity;
  const bool has_intensity = fieldReader(cloud, "intensity", intensity);

  const std::size_t width = cloud.width;
  const std::size_t height = cloud.height;
  PointCloudReadStats stats;
  stats.total = width * height;
  if (stats.total == 0) {
    return stats;
  }
  if (static_cast<std::size_t>(cloud.point_step) * width > cloud.row_step) {
    throw std::invalid_argument("PointCloud2 row_step is smaller than width * point_step");
  }
  if (static_cast<std::size_t>(cloud.row_step) * (height - 1) +
    static_cast<std::size_t>(cloud.point_step) * width > cloud.data.size())
  {
    throw std::invalid_argument("PointCloud2 data buffer is smaller than its declared layout");
  }

  const bool swap = cloud.is_bigendian != hostIsBigEndian();
  points.reserve(stats.total);
  for (std::size_t row = 0; row < height; ++row) {
    const std::uint8_t * point = cloud.data.data() + row * cloud.row_step;
    for (std::size_t column = 0; column < width; ++column, point += cloud.point_step) {
      PointXYZI output;
      output.position = Eigen::Vector3d(x.read(point, swap), y.read(point, swap),
          z.read(point, swap));
      output.intensity = has_intensity ? intensity.read(point, swap) : 0.0;
      if (!output.position.allFinite() || !std::isfinite(output.intensity)) {
        ++stats.non_finite;
        continue;
      }
      points.push_back(output);
    }
  }
  return stats;
}

std::vector<PointXYZI> readPointCloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<PointXYZI> points;
  readPointCloud(cloud, points);
  return points;
}

void writePointCloud(
  const std::vector<PointXYZI> & points,
  const std_msgs::msg::Header & header,
  sensor_msgs::msg::PointCloud2 & cloud)
{
  constexpr std::uint32_t kPointStep = 4 * sizeof(float);
  cloud.header = header;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.is_bigendian = hostIsBigEndian();
  cloud.is_dense = true;
  cloud.point_step = kPointStep;
  cloud.row_step = kPointStep * cloud.width;
  cloud.fields.clear();
  appendFloat32Field(cloud, "x", 0);
  appendFloat32Field(cloud, "y", 4);
  appendFloat32Field(cloud, "z", 8);
  appendFloat32Field(cloud, "intensity", 12);
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
  std::uint8_t * data = cloud.data.data();
  for (const auto & point : points) {
    const float values[4] = {
      static_cast<float>(point.position.x()), static_cast<float>(point.position.y()),
      static_cast<float>(point.position.z()), static_cast<float>(point.intensity)};
    std::memcpy(data, values, kPointStep);
    data += kPointStep;
  }
}

}  // namespace perception_core
