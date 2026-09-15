from fusion_interfaces.msg import Detection2DArray
from fusion_interfaces.msg import FusedDetection
from fusion_interfaces.msg import FusedDetectionArray
from fusion_interfaces.msg import SyncStatus


def test_mvp_interfaces_are_typed():
    assert Detection2DArray.get_fields_and_field_types() == {
        'header': 'std_msgs/Header',
        'inference_ms': 'float',
        'detections': 'sequence<fusion_interfaces/Detection2D>',
    }
    assert FusedDetection.get_fields_and_field_types() == {
        'header': 'std_msgs/Header',
        'detection': 'fusion_interfaces/Detection2D',
        'position': 'geometry_msgs/Point',
        'depth': 'float',
        'lidar_point_count': 'uint32',
        'valid': 'boolean',
    }
    assert FusedDetectionArray.get_fields_and_field_types() == {
        'header': 'std_msgs/Header',
        'detections': 'sequence<fusion_interfaces/FusedDetection>',
    }
    assert 'camera_lidar_offset_ms' in SyncStatus.get_fields_and_field_types()
