import cv2
import numpy as np

from yolo_detector.color_detector import detect_color_blobs


def test_detects_orange_blob_and_ignores_small_noise():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (120, 60), (200, 210), (0, 140, 255), -1)
    cv2.rectangle(image, (5, 5), (8, 8), (0, 140, 255), -1)

    detections = detect_color_blobs(
        image, (3, 80, 60), (35, 255, 255), min_area=150, padding=4)

    assert len(detections) == 1
    assert detections[0].label == 'worker'
    assert detections[0].bbox == (116, 56, 205, 215)


def test_empty_image_has_no_detection():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    assert detect_color_blobs(image, (3, 80, 60), (35, 255, 255)) == []
