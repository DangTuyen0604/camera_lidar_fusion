from builtin_interfaces.msg import Time

from yolo_detector.camera_viewer_node import frame_filename


def test_frame_names_sort_in_time_order():
    earlier = frame_filename(Time(sec=9, nanosec=999999999))
    later = frame_filename(Time(sec=10, nanosec=0))

    assert earlier == 'frame_0000000009_999999999.png'
    assert sorted([later, earlier]) == [earlier, later]
