import numpy as np

from yolo_detector.model_loader import letterbox, postprocess_yolov8


def test_letterbox_preserves_aspect_ratio():
    image = np.zeros((100, 200, 3), dtype=np.uint8)

    tensor, scale, pad_x, pad_y = letterbox(image, 640, 640)

    assert tensor.shape == (1, 3, 640, 640)
    assert scale == 3.2
    assert pad_x == 0
    assert pad_y == 160


def test_postprocess_yolov8_maps_box_to_original_image():
    output = np.zeros((1, 6, 2), dtype=np.float32)
    output[0, :4, 0] = [320.0, 320.0, 320.0, 160.0]
    output[0, 4:, 0] = [0.1, 0.9]

    detections = postprocess_yolov8(
        output,
        {0: 'person', 1: 'car'},
        image_width=200,
        image_height=100,
        scale=3.2,
        pad_x=0,
        pad_y=160,
        confidence_threshold=0.25,
        iou_threshold=0.45,
    )

    assert len(detections) == 1
    assert detections[0].label == 'car'
    assert detections[0].bbox == (50, 25, 150, 75)
