import cv2
import numpy as np


def _draw_label(image, text, x, y, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1
    (text_width, text_height), _ = cv2.getTextSize(
        text,
        font,
        font_scale,
        thickness,
    )
    top = max(0, y - text_height - 8)
    cv2.rectangle(
        image,
        (x, top),
        (min(image.shape[1] - 1, x + text_width + 6), y),
        color,
        -1,
    )
    cv2.putText(
        image,
        text,
        (x + 3, max(text_height + 1, y - 4)),
        font,
        font_scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA,
    )


def draw_detections(image, detections):
    output = image.copy()

    for detection in detections:
        x1, y1, x2, y2 = detection.bbox
        color = (0, 180, 255)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        _draw_label(
            output,
            f'{detection.label} {detection.confidence:.2f}',
            x1,
            y1,
            color,
        )

    return output


def draw_fused_detections(image, detections, fused_detections):
    output = draw_detections(image, detections)

    for fused in fused_detections:
        x1, y1, x2, y2 = fused.detection.bbox
        color = (0, 220, 0)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)

        pixels = np.rint(fused.selected_pixels).astype(np.int32)

        for pixel_x, pixel_y in pixels:
            cv2.circle(
                output,
                (int(pixel_x), int(pixel_y)),
                2,
                (255, 0, 255),
                -1,
            )

        _draw_label(
            output,
            f'{fused.detection.label} '
            f'{fused.detection.confidence:.2f} '
            f'{fused.distance_m:.1f}m '
            f'({fused.point_count} pts)',
            x1,
            y1,
            color,
        )

    return output
