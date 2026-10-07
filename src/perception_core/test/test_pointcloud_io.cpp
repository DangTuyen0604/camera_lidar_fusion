#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "perception_core/pointcloud_io.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

struct FieldSpec
{
  std::string name;
  std::uint8_t datatype;
  std::uint32_t offset;
};

// Builds a cloud whose byte layout is written by hand, independent of the
// reader under test.
class CloudBuilder
{
public:
  CloudBuilder(std::vector<FieldSpec> fields, std::uint32_t point_step, bool big_endian = false)
  : big_endian_(big_endian)
  {
    cloud_.point_step = point_step;
    cloud_.is_bigendian = big_endian;
    cloud_.height = 1;
    for (const auto & spec : fields) {
      PointField field;
      field.name = spec.name;
      field.datatype = spec.datatype;
      field.offset = spec.offset;
      field.count = 1;
      cloud_.fields.push_back(field);
    }
  }

  template<typename T>
  void put(std::size_t point, std::uint32_t offset, T value)
  {
    const std::size_t needed = (point + 1) * cloud_.point_step;
    if (cloud_.data.size() < needed) {
      cloud_.data.resize(needed, 0);
      cloud_.width = static_cast<std::uint32_t>(needed / cloud_.point_step);
      cloud_.row_step = cloud_.width * cloud_.point_step;
    }
    std::uint8_t bytes[sizeof(T)];
    std::memcpy(bytes, &value, sizeof(T));
    if (big_endian_) {
      std::reverse(bytes, bytes + sizeof(T));
    }
    std::memcpy(cloud_.data.data() + point * cloud_.point_step + offset, bytes, sizeof(T));
  }

  PointCloud2 cloud() const {return cloud_;}

private:
  PointCloud2 cloud_;
  bool big_endian_;
};

std::vector<FieldSpec> xyzFloat32()
{
  return {{"x", PointField::FLOAT32, 0}, {"y", PointField::FLOAT32, 4},
    {"z", PointField::FLOAT32, 8}};
}

}  // namespace

TEST(PointCloudIo, ReadsFloat32Intensity)
{
  auto fields = xyzFloat32();
  fields.push_back({"intensity", PointField::FLOAT32, 12});
  CloudBuilder builder(fields, 16);
  builder.put<float>(0, 0, 1.5F);
  builder.put<float>(0, 4, -2.0F);
  builder.put<float>(0, 8, 0.25F);
  builder.put<float>(0, 12, 0.75F);

  const auto points = perception_core::readPointCloud(builder.cloud());
  ASSERT_EQ(points.size(), 1U);
  EXPECT_DOUBLE_EQ(points[0].position.x(), 1.5);
  EXPECT_DOUBLE_EQ(points[0].position.y(), -2.0);
  EXPECT_DOUBLE_EQ(points[0].position.z(), 0.25);
  EXPECT_DOUBLE_EQ(points[0].intensity, 0.75);
}

TEST(PointCloudIo, ReadsIntegerIntensityOfHesaiAndRobosenseDrivers)
{
  for (const std::uint8_t datatype : {PointField::UINT8, PointField::UINT16}) {
    auto fields = xyzFloat32();
    fields.push_back({"intensity", datatype, 12});
    CloudBuilder builder(fields, 16);
    builder.put<float>(0, 0, 1.0F);
    if (datatype == PointField::UINT8) {
      builder.put<std::uint8_t>(0, 12, 200);
    } else {
      builder.put<std::uint16_t>(0, 12, 40000);
    }

    const auto points = perception_core::readPointCloud(builder.cloud());
    ASSERT_EQ(points.size(), 1U);
    EXPECT_DOUBLE_EQ(points[0].intensity, datatype == PointField::UINT8 ? 200.0 : 40000.0);
  }
}

TEST(PointCloudIo, MissingIntensityReadsAsZero)
{
  CloudBuilder builder(xyzFloat32(), 12);
  builder.put<float>(0, 0, 3.0F);
  const auto points = perception_core::readPointCloud(builder.cloud());
  ASSERT_EQ(points.size(), 1U);
  EXPECT_DOUBLE_EQ(points[0].position.x(), 3.0);
  EXPECT_DOUBLE_EQ(points[0].intensity, 0.0);
}

TEST(PointCloudIo, ReadsFloat64CoordinatesAndBigEndianData)
{
  CloudBuilder builder({{"x", PointField::FLOAT64, 0}, {"y", PointField::FLOAT64, 8},
      {"z", PointField::FLOAT64, 16}, {"intensity", PointField::UINT16, 24}}, 26, true);
  builder.put<double>(0, 0, 12.5);
  builder.put<double>(0, 8, -0.125);
  builder.put<double>(0, 16, 1.0e3);
  builder.put<std::uint16_t>(0, 24, 513);

  const auto points = perception_core::readPointCloud(builder.cloud());
  ASSERT_EQ(points.size(), 1U);
  EXPECT_DOUBLE_EQ(points[0].position.x(), 12.5);
  EXPECT_DOUBLE_EQ(points[0].position.y(), -0.125);
  EXPECT_DOUBLE_EQ(points[0].position.z(), 1.0e3);
  EXPECT_DOUBLE_EQ(points[0].intensity, 513.0);
}

TEST(PointCloudIo, SkipsAndCountsNonFinitePoints)
{
  auto fields = xyzFloat32();
  fields.push_back({"intensity", PointField::FLOAT32, 12});
  CloudBuilder builder(fields, 16);
  builder.put<float>(0, 0, std::numeric_limits<float>::quiet_NaN());
  builder.put<float>(1, 4, std::numeric_limits<float>::infinity());
  builder.put<float>(2, 12, std::numeric_limits<float>::quiet_NaN());
  builder.put<float>(3, 0, 7.0F);

  std::vector<perception_core::PointXYZI> points;
  const auto stats = perception_core::readPointCloud(builder.cloud(), points);
  EXPECT_EQ(stats.total, 4U);
  EXPECT_EQ(stats.non_finite, 3U);
  ASSERT_EQ(points.size(), 1U);
  EXPECT_DOUBLE_EQ(points[0].position.x(), 7.0);
}

TEST(PointCloudIo, HonoursRowStepPaddingOfOrganizedClouds)
{
  // 2 x 2 organized cloud, each row padded by 8 unused bytes.
  PointCloud2 cloud;
  for (const auto & spec : xyzFloat32()) {
    PointField field;
    field.name = spec.name;
    field.datatype = spec.datatype;
    field.offset = spec.offset;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.width = 2;
  cloud.height = 2;
  cloud.point_step = 12;
  cloud.row_step = 2 * 12 + 8;
  cloud.data.assign(cloud.row_step * cloud.height, 0xFF);  // padding is garbage
  for (std::uint32_t row = 0; row < 2; ++row) {
    for (std::uint32_t column = 0; column < 2; ++column) {
      const float xyz[3] = {static_cast<float>(row * 10 + column), 0.0F, 0.0F};
      std::memcpy(cloud.data.data() + row * cloud.row_step + column * 12, xyz, sizeof(xyz));
    }
  }

  const auto points = perception_core::readPointCloud(cloud);
  ASSERT_EQ(points.size(), 4U);
  EXPECT_DOUBLE_EQ(points[2].position.x(), 10.0);
  EXPECT_DOUBLE_EQ(points[3].position.x(), 11.0);
}

TEST(PointCloudIo, ReusesOutputBufferCapacity)
{
  CloudBuilder builder(xyzFloat32(), 12);
  for (std::size_t index = 0; index < 100; ++index) {
    builder.put<float>(index, 0, static_cast<float>(index));
  }
  std::vector<perception_core::PointXYZI> points;
  perception_core::readPointCloud(builder.cloud(), points);
  const auto * storage = points.data();
  perception_core::readPointCloud(builder.cloud(), points);
  EXPECT_EQ(points.data(), storage);
  EXPECT_EQ(points.size(), 100U);
}

TEST(PointCloudIo, RejectsMalformedLayouts)
{
  CloudBuilder missing_z({{"x", PointField::FLOAT32, 0}, {"y", PointField::FLOAT32, 4}}, 8);
  missing_z.put<float>(0, 0, 1.0F);
  EXPECT_THROW(perception_core::readPointCloud(missing_z.cloud()), std::invalid_argument);

  CloudBuilder overflowing({{"x", PointField::FLOAT32, 0}, {"y", PointField::FLOAT32, 4},
      {"z", PointField::FLOAT64, 8}}, 12);
  overflowing.put<float>(0, 0, 1.0F);
  EXPECT_THROW(perception_core::readPointCloud(overflowing.cloud()), std::invalid_argument);

  CloudBuilder short_buffer(xyzFloat32(), 12);
  short_buffer.put<float>(1, 0, 1.0F);
  auto cloud = short_buffer.cloud();
  cloud.data.resize(cloud.data.size() - 1);
  EXPECT_THROW(perception_core::readPointCloud(cloud), std::invalid_argument);
}

TEST(PointCloudIo, WriteThenReadRoundTrips)
{
  const std::vector<perception_core::PointXYZI> input{
    {Eigen::Vector3d(1.0, 2.0, 3.0), 4.0},
    {Eigen::Vector3d(-0.5, 0.25, 8.0), 0.0}};
  std_msgs::msg::Header header;
  header.frame_id = "base_link";
  PointCloud2 cloud;
  perception_core::writePointCloud(input, header, cloud);
  EXPECT_EQ(cloud.header.frame_id, "base_link");
  EXPECT_EQ(cloud.width, 2U);

  const auto output = perception_core::readPointCloud(cloud);
  ASSERT_EQ(output.size(), input.size());
  for (std::size_t index = 0; index < input.size(); ++index) {
    EXPECT_TRUE(output[index].position.isApprox(input[index].position));
    EXPECT_DOUBLE_EQ(output[index].intensity, input[index].intensity);
  }
}
