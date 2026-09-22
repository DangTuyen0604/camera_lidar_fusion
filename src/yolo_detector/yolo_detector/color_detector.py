"""Deterministic color-blob detector for simulated safety targets."""

import cv2
import numpy as np

from yolo_detector.detection_converter import Detection


def detect_color_blobs(
        image, lower_hsv, upper_hsv, min_area=150, padding=4,
        label='worker', class_id=0):
    """Return typed detections for sufficiently large HSV color regions."""
    if image is None or image.size == 0:
        return []
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv, np.asarray(lower_hsv, dtype=np.uint8),
        np.asarray(upper_hsv, dtype=np.uint8))
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    count, unused, stats, unused_centroids = cv2.connectedComponentsWithStats(
        mask, connectivity=8)
    height, width = image.shape[:2]
    detections = []
    for index in range(1, count):
        x, y, box_width, box_height, area = stats[index]
        if area < min_area:
            continue
        x_min = max(0, int(x) - padding)
        y_min = max(0, int(y) - padding)
        x_max = min(width - 1, int(x + box_width) + padding)
        y_max = min(height - 1, int(y + box_height) + padding)
        confidence = min(0.99, 0.80 + float(area) / float(width * height))
        detections.append(Detection(
            class_id=class_id,
            label=label,
            confidence=confidence,
            bbox=(x_min, y_min, x_max, y_max),
        ))
    return sorted(detections, key=lambda item: item.confidence, reverse=True)
