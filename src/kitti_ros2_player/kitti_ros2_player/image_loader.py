from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2


NANOSECONDS_PER_SECOND = 1_000_000_000


def parse_kitti_timestamp(value):
    value = value.strip()
    try:
        date_part, fractional_part = value.split('.', 1)
        parsed_date = datetime.strptime(
            date_part,
            '%Y-%m-%d %H:%M:%S',
        ).replace(tzinfo=timezone.utc)
    except ValueError as error:
        raise RuntimeError(f'Invalid KITTI timestamp: {value}') from error

    if not fractional_part.isdigit() or len(fractional_part) > 9:
        raise RuntimeError(f'Invalid KITTI timestamp: {value}')

    return (
        int(parsed_date.timestamp()) * NANOSECONDS_PER_SECOND
        + int(fractional_part.ljust(9, '0'))
    )


def load_kitti_timestamps(timestamp_path):
    timestamp_path = Path(timestamp_path).expanduser()
    if not timestamp_path.is_file():
        raise RuntimeError(f'Timestamp file not found: {timestamp_path}')

    timestamps = [
        parse_kitti_timestamp(line)
        for line in timestamp_path.read_text(encoding='utf-8').splitlines()
        if line.strip()
    ]
    if not timestamps:
        raise RuntimeError(f'No timestamps found in: {timestamp_path}')
    if any(current <= previous for previous, current in zip(
        timestamps,
        timestamps[1:],
    )):
        raise RuntimeError(
            f'Timestamps must be strictly increasing: {timestamp_path}'
        )
    return timestamps


@dataclass(frozen=True)
class TimestampSequence:
    image_timestamps_ns: tuple
    lidar_timestamps_ns: tuple

    @classmethod
    def load(cls, image_path, lidar_path):
        image_timestamps = tuple(load_kitti_timestamps(image_path))
        lidar_timestamps = tuple(load_kitti_timestamps(lidar_path))
        if len(image_timestamps) != len(lidar_timestamps):
            raise RuntimeError(
                'Image and LiDAR timestamp counts do not match: '
                f'images={len(image_timestamps)}, '
                f'lidar={len(lidar_timestamps)}'
            )
        return cls(image_timestamps, lidar_timestamps)

    def __len__(self):
        return len(self.image_timestamps_ns)

    def sensor_offset_ns(self, frame_index):
        return (
            self.image_timestamps_ns[frame_index]
            - self.lidar_timestamps_ns[frame_index]
        )

    @property
    def max_absolute_sensor_offset_ns(self):
        return max(
            abs(image - lidar)
            for image, lidar in zip(
                self.image_timestamps_ns,
                self.lidar_timestamps_ns,
            )
        )

    @property
    def cycle_duration_ns(self):
        if len(self) == 1:
            return 100_000_000
        intervals = sorted(
            current - previous
            for previous, current in zip(
                self.image_timestamps_ns,
                self.image_timestamps_ns[1:],
            )
        )
        return (
            self.image_timestamps_ns[-1]
            - self.image_timestamps_ns[0]
            + intervals[len(intervals) // 2]
        )

    def rebased_timestamp_ns(self, frame_index, cycle_index, ros_start_ns):
        dataset_elapsed = (
            self.image_timestamps_ns[frame_index]
            - self.image_timestamps_ns[0]
        )
        return (
            ros_start_ns
            + cycle_index * self.cycle_duration_ns
            + dataset_elapsed
        )


class ImageLoader:

    def __init__(self, image_dir, loop=True):
        self.image_dir = Path(image_dir).expanduser()
        self.loop = loop

        if not self.image_dir.is_dir():
            raise RuntimeError(
                f'KITTI image directory not found: {self.image_dir}'
            )

        self.image_paths = sorted(self.image_dir.glob('*.png'))

        if not self.image_paths:
            raise RuntimeError(
                f'No KITTI PNG images found in: {self.image_dir}'
            )

        self.frame_index = 0

    def __len__(self):
        return len(self.image_paths)

    def next_image(self):
        if self.frame_index >= len(self.image_paths):
            if not self.loop:
                raise StopIteration
            self.frame_index = 0

        path = self.image_paths[self.frame_index]
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)

        if image is None:
            raise RuntimeError(f'Cannot decode KITTI image: {path}')

        self.frame_index += 1

        if self.loop and self.frame_index >= len(self.image_paths):
            self.frame_index = 0

        return image, path
