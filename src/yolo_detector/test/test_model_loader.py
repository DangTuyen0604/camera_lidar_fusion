import numpy as np
import pytest

from yolo_detector.model_loader import letterbox, postprocess_yolov8


def test_letterbox_preserves_aspect_ratio():
    image = np.zeros((100, 200, 3), dtype=np.uint8)

    tensor, scale, pad_x, pad_y = letterbox(image, 640, 640)

    assert tensor.shape == (1, 3, 640, 640)
    assert scale == 3.2
    assert pad_x == 0
    assert pad_y == 160


def test_letterbox_rejects_empty_image():
    with pytest.raises(RuntimeError, match='non-empty BGR'):
        letterbox(np.empty((0, 0, 3), dtype=np.uint8), 640, 640)


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


def test_postprocess_applies_class_aware_nms():
    output = np.zeros((1, 6, 3), dtype=np.float32)
    output[0, :4, 0] = [320.0, 320.0, 200.0, 200.0]
    output[0, 4:, 0] = [0.9, 0.1]
    output[0, :4, 1] = [325.0, 325.0, 200.0, 200.0]
    output[0, 4:, 1] = [0.8, 0.1]
    output[0, :4, 2] = [320.0, 320.0, 200.0, 200.0]
    output[0, 4:, 2] = [0.1, 0.85]

    detections = postprocess_yolov8(
        output,
        {0: 'person', 1: 'car'},
        image_width=640,
        image_height=640,
        scale=1.0,
        pad_x=0,
        pad_y=0,
        confidence_threshold=0.25,
        iou_threshold=0.45,
    )

    assert [item.label for item in detections] == ['person', 'car']


def test_postprocess_rejects_wrong_shape_and_non_contiguous_classes():
    with pytest.raises(RuntimeError, match='Unexpected YOLO output shape'):
        postprocess_yolov8(
            np.zeros((1, 2, 3, 4), dtype=np.float32),
            {0: 'person'},
            640, 640, 1.0, 0, 0, 0.25, 0.45,
        )
    with pytest.raises(RuntimeError, match='contiguous'):
        postprocess_yolov8(
            np.zeros((1, 6, 1), dtype=np.float32),
            {0: 'person', 2: 'car'},
            640, 640, 1.0, 0, 0, 0.25, 0.45,
        )
