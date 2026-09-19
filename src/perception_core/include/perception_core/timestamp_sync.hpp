#ifndef PERCEPTION_CORE__TIMESTAMP_SYNC_HPP_
#define PERCEPTION_CORE__TIMESTAMP_SYNC_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace perception_core
{
enum class SensorStream : std::size_t {Image = 0, CameraInfo = 1, PointCloud = 2};
enum class SyncFault : std::uint8_t {None = 0, Delayed = 1, OutOfOrder = 2, WrongFrame = 3};

struct TimestampSyncCounters
{
  std::uint64_t synchronized_pairs{0};
  std::uint64_t dropped_messages{0};
  std::uint64_t out_of_order_messages{0};
  std::uint64_t wrong_frame_messages{0};
};

class TimestampSyncMonitor
{
public:
  TimestampSyncMonitor(
    double tolerance_ms, std::size_t queue_size,
    std::array<std::string, 3> expected_frames);
  SyncFault observe(SensorStream stream, std::int64_t stamp_ns, const std::string & frame_id);
  SyncFault observeMatch(
    std::int64_t image_ns, std::int64_t camera_info_ns,
    std::int64_t pointcloud_ns);
  double latestCameraLidarOffsetMs() const noexcept;
  const TimestampSyncCounters & counters() const noexcept;

private:
  double tolerance_ms_;
  std::size_t queue_size_;
  std::array<std::string, 3> expected_frames_;
  std::array<std::int64_t, 3> last_stamp_ns_{{-1, -1, -1}};
  std::array<std::uint64_t, 3> accepted_{{0, 0, 0}};
  double latest_offset_ms_{0.0};
  TimestampSyncCounters counters_;
};
}  // namespace perception_core
#endif  // PERCEPTION_CORE__TIMESTAMP_SYNC_HPP_
