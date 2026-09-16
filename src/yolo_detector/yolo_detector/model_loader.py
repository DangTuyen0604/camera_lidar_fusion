import ast
from pathlib import Path

import cv2
import numpy as np

from yolo_detector.detection_converter import Detection


def _load_onnxruntime():
    try:
        import onnxruntime
    except ImportError as error:
        raise RuntimeError(
            'ONNX Runtime is not available. Install requirements.txt.'
        ) from error

    return onnxruntime


def letterbox(image, input_width, input_height):
    image = np.asarray(image)
    if (
        image.ndim != 3
        or image.shape[2] != 3
        or image.shape[0] == 0
        or image.shape[1] == 0
    ):
        raise RuntimeError(
            f'Expected non-empty BGR image shape (H, W, 3), got {image.shape}'
        )
    if input_width <= 0 or input_height <= 0:
        raise RuntimeError(
            f'YOLO input dimensions must be positive: '
            f'{input_width}x{input_height}'
        )
    image_height, image_width = image.shape[:2]
    scale = min(
        input_width / image_width,
        input_height / image_height,
    )
    resized_width = int(round(image_width * scale))
    resized_height = int(round(image_height * scale))
    pad_x = (input_width - resized_width) // 2
    pad_y = (input_height - resized_height) // 2

    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )
    padded = np.full(
        (input_height, input_width, 3),
        114,
        dtype=np.uint8,
    )
    padded[
        pad_y:pad_y + resized_height,
        pad_x:pad_x + resized_width,
    ] = resized

    input_tensor = np.ascontiguousarray(
        padded[:, :, ::-1].transpose(2, 0, 1)[None],
        dtype=np.float32,
    )
    input_tensor /= 255.0

    return input_tensor, scale, pad_x, pad_y


def postprocess_yolov8(
    output,
    class_names,
    image_width,
    image_height,
    scale,
    pad_x,
    pad_y,
    confidence_threshold,
    iou_threshold,
):
    output = np.asarray(output)
    if not np.isfinite(output).all():
        raise RuntimeError('YOLO output contains NaN or Inf values')

    if output.ndim == 3 and output.shape[0] == 1:
        predictions = output[0]
    elif output.ndim == 2:
        predictions = output
    else:
        raise RuntimeError(
            f'Unexpected YOLO output shape: {output.shape}'
        )

    if not class_names:
        raise RuntimeError('YOLO class mapping must not be empty')
    expected_class_ids = set(range(len(class_names)))
    if set(class_names) != expected_class_ids:
        raise RuntimeError(
            'YOLO class IDs must be contiguous and start at zero'
        )
    if not 0.0 <= confidence_threshold <= 1.0:
        raise RuntimeError('confidence_threshold must be between 0 and 1')
    if not 0.0 <= iou_threshold <= 1.0:
        raise RuntimeError('iou_threshold must be between 0 and 1')
    if scale <= 0.0:
        raise RuntimeError('Letterbox scale must be positive')

    expected_columns = 4 + len(class_names)

    if predictions.shape[0] == expected_columns:
        predictions = predictions.T

    if predictions.shape[1] != expected_columns:
        raise RuntimeError(
            'YOLO output does not match model classes: '
            f'output={predictions.shape}, classes={len(class_names)}'
        )

    class_scores = predictions[:, 4:]
    class_ids = np.argmax(class_scores, axis=1)
    confidences = np.max(class_scores, axis=1)
    candidates = confidences >= confidence_threshold
    predictions = predictions[candidates]
    class_ids = class_ids[candidates]
    confidences = confidences[candidates]

    boxes_xywh = []
    valid_class_ids = []
    valid_confidences = []

    for prediction, class_id, confidence in zip(
        predictions,
        class_ids,
        confidences,
    ):
        center_x, center_y, box_width, box_height = prediction[:4]
        x1 = (center_x - box_width / 2.0 - pad_x) / scale
        y1 = (center_y - box_height / 2.0 - pad_y) / scale
        x2 = (center_x + box_width / 2.0 - pad_x) / scale
        y2 = (center_y + box_height / 2.0 - pad_y) / scale
        x1 = int(np.clip(round(x1), 0, image_width - 1))
        y1 = int(np.clip(round(y1), 0, image_height - 1))
        x2 = int(np.clip(round(x2), 0, image_width - 1))
        y2 = int(np.clip(round(y2), 0, image_height - 1))

        if x2 <= x1 or y2 <= y1:
            continue

        boxes_xywh.append([x1, y1, x2 - x1, y2 - y1])
        valid_class_ids.append(int(class_id))
        valid_confidences.append(float(confidence))

    kept_indices = []

    for class_id in sorted(set(valid_class_ids)):
        class_indices = [
            index
            for index, value in enumerate(valid_class_ids)
            if value == class_id
        ]
        class_boxes = [boxes_xywh[index] for index in class_indices]
        class_confidences = [
            valid_confidences[index] for index in class_indices
        ]
        nms_indices = cv2.dnn.NMSBoxes(
            class_boxes,
            class_confidences,
            confidence_threshold,
            iou_threshold,
        )

        for nms_index in np.asarray(nms_indices).reshape(-1):
            kept_indices.append(class_indices[int(nms_index)])

    detections = []

    for index in kept_indices:
        x, y, width, height = boxes_xywh[index]
        class_id = valid_class_ids[index]
        detections.append(Detection(
            class_id=class_id,
            label=class_names[class_id],
            confidence=valid_confidences[index],
            bbox=(x, y, x + width, y + height),
        ))

    return sorted(
        detections,
        key=lambda detection: detection.confidence,
        reverse=True,
    )


class YoloOnnxDetector:

    def __init__(
        self,
        model_path,
        confidence_threshold=0.25,
        iou_threshold=0.45,
    ):
        model_path = Path(model_path).expanduser()

        if not model_path.is_file():
            raise RuntimeError(f'YOLO model not found: {model_path}')

        if not 0.0 <= confidence_threshold <= 1.0:
            raise RuntimeError(
                'confidence_threshold must be between 0 and 1'
            )
        if not 0.0 <= iou_threshold <= 1.0:
            raise RuntimeError('iou_threshold must be between 0 and 1')

        onnxruntime = _load_onnxruntime()
        try:
            self.session = onnxruntime.InferenceSession(
                str(model_path),
                providers=['CPUExecutionProvider'],
            )
        except Exception as error:
            raise RuntimeError(
                f'Cannot load YOLO ONNX model: {model_path}: {error}'
            ) from error
        if len(self.session.get_inputs()) != 1:
            raise RuntimeError('YOLO model must have exactly one input')
        if len(self.session.get_outputs()) != 1:
            raise RuntimeError('YOLO model must have exactly one output')
        model_input = self.session.get_inputs()[0]
        input_shape = model_input.shape

        if len(input_shape) != 4:
            raise RuntimeError(
                f'Unexpected YOLO input shape: {input_shape}'
            )

        if model_input.type != 'tensor(float)':
            raise RuntimeError(
                f'YOLO input must be float32, got {model_input.type}'
            )
        try:
            self.input_height = int(input_shape[2])
            self.input_width = int(input_shape[3])
        except (TypeError, ValueError) as error:
            raise RuntimeError(
                f'YOLO input must have static spatial dimensions: '
                f'{input_shape}'
            ) from error
        if self.input_height <= 0 or self.input_width <= 0:
            raise RuntimeError(
                f'Invalid YOLO input dimensions: {input_shape}'
            )

        model_output = self.session.get_outputs()[0]
        output_shape = model_output.shape
        if len(output_shape) != 3:
            raise RuntimeError(
                f'Unexpected YOLO output shape: {output_shape}'
            )

        self.input_name = model_input.name
        metadata = self.session.get_modelmeta().custom_metadata_map
        try:
            names = ast.literal_eval(metadata.get('names', '{}'))
        except (SyntaxError, ValueError) as error:
            raise RuntimeError('Invalid YOLO class metadata') from error

        if not names:
            raise RuntimeError('YOLO model metadata has no class names')
        if isinstance(names, (list, tuple)):
            names = dict(enumerate(names))
        if not isinstance(names, dict):
            raise RuntimeError('YOLO class metadata must be a map or list')
        try:
            self.class_names = {
                int(class_id): str(label)
                for class_id, label in names.items()
            }
        except (TypeError, ValueError) as error:
            raise RuntimeError('Invalid YOLO class metadata') from error
        if set(self.class_names) != set(range(len(self.class_names))):
            raise RuntimeError(
                'YOLO class IDs must be contiguous and start at zero'
            )

        expected_columns = 4 + len(self.class_names)
        static_output_dimensions = {
            int(value)
            for value in output_shape[1:]
            if isinstance(value, int)
        }
        if static_output_dimensions and expected_columns not in static_output_dimensions:
            raise RuntimeError(
                'YOLO output does not match model classes: '
                f'output={output_shape}, classes={len(self.class_names)}'
            )
        self.confidence_threshold = confidence_threshold
        self.iou_threshold = iou_threshold

    def detect(self, image):
        image_height, image_width = image.shape[:2]
        input_tensor, scale, pad_x, pad_y = letterbox(
            image,
            self.input_width,
            self.input_height,
        )
        output = self.session.run(
            None,
            {self.input_name: input_tensor},
        )[0]

        return postprocess_yolov8(
            output,
            self.class_names,
            image_width,
            image_height,
            scale,
            pad_x,
            pad_y,
            self.confidence_threshold,
            self.iou_threshold,
        )
