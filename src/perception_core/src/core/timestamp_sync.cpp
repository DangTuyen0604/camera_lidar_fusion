#include "perception_core/timestamp_sync.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace perception_core
{
TimestampSyncMonitor::TimestampSyncMonitor(
  double tolerance_ms, std::size_t queue_size, std::array<std::string, 3> expected_frames)
: tolerance_ms_(tolerance_ms), queue_size_(queue_size), expected_frames_(std::move(expected_frames))
{
  if (!std::isfinite(tolerance_ms_) || tolerance_ms_ < 0.0 || queue_size_ == 0) {
    throw std::invalid_argument("Invalid timestamp synchronizer configuration");
  }
}

SyncFault TimestampSyncMonitor::observe(
  SensorStream stream, std::int64_t stamp_ns, const std::string & frame_id)
{
  const auto index = static_cast<std::size_t>(stream);
  if (!expected_frames_[index].empty() && frame_id != expected_frames_[index]) {
    ++counters_.wrong_frame_messages;
    ++counters_.dropped_messages;
    return SyncFault::WrongFrame;
  }
  if (last_stamp_ns_[index] >= 0 && stamp_ns <= last_stamp_ns_[index]) {
    ++counters_.out_of_order_messages;
    ++counters_.dropped_messages;
    return SyncFault::OutOfOrder;
  }
  last_stamp_ns_[index] = stamp_ns;
  ++accepted_[index];
  const auto minimum_count = *std::min_element(accepted_.begin(), accepted_.end());
  if (accepted_[index] > minimum_count + queue_size_) {
    ++counters_.dropped_messages;
    return SyncFault::Delayed;
  }
  if (std::all_of(last_stamp_ns_.begin(), last_stamp_ns_.end(), [](auto value) {
      return value >= 0;
    }) &&
    *std::min_element(accepted_.begin(), accepted_.end()) ==
    *std::max_element(accepted_.begin(), accepted_.end()))
  {
    latest_offset_ms_ = static_cast<double>(last_stamp_ns_[2] - last_stamp_ns_[0]) / 1.0e6;
    const auto [minimum_stamp, maximum_stamp] = std::minmax_element(
      last_stamp_ns_.begin(), last_stamp_ns_.end());
    if (static_cast<double>(*maximum_stamp - *minimum_stamp) / 1.0e6 > tolerance_ms_) {
      ++counters_.dropped_messages;
      return SyncFault::Delayed;
    }
  }
  return SyncFault::None;
}

SyncFault TimestampSyncMonitor::observeMatch(
  std::int64_t image_ns, std::int64_t camera_info_ns, std::int64_t pointcloud_ns)
{
  const auto minimum = std::min({image_ns, camera_info_ns, pointcloud_ns});
  const auto maximum = std::max({image_ns, camera_info_ns, pointcloud_ns});
  latest_offset_ms_ = static_cast<double>(pointcloud_ns - image_ns) / 1.0e6;
  if (static_cast<double>(maximum - minimum) / 1.0e6 > tolerance_ms_) {
    ++counters_.dropped_messages;
    return SyncFault::Delayed;
  }
  ++counters_.synchronized_pairs;
  return SyncFault::None;
}

double TimestampSyncMonitor::latestCameraLidarOffsetMs() const noexcept {return latest_offset_ms_;}
const TimestampSyncCounters & TimestampSyncMonitor::counters() const noexcept {return counters_;}
}  // namespace perception_core
