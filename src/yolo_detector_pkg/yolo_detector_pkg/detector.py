import os

from yolo_detector_pkg.class_names import get_class_name
from yolo_detector_pkg.class_names import get_num_classes
from yolo_detector_pkg.models import Detection
from yolo_detector_pkg.postprocess import decode_yolo_output
from yolo_detector_pkg.preprocess import preprocess_image


class YoloDetector:
    def __init__(self, model_path: str, input_width: int, input_height: int, conf_threshold: float):
        self.model_path = model_path
        self.input_width = input_width
        self.input_height = input_height
        self.conf_threshold = conf_threshold
        self.backend = None

    def load(self):
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f'Model not found: {self.model_path}')

        from yolo_detector_pkg.onnx_infer import OnnxObjectDetector

        self.backend = OnnxObjectDetector(self.model_path)

    def is_loaded(self) -> bool:
        return self.backend is not None

    def set_conf_threshold(self, conf_threshold: float):
        self.conf_threshold = conf_threshold

    def detect(self, cv_image) -> list[Detection]:
        if self.backend is None:
            raise RuntimeError('Detector backend is not loaded')

        input_tensor, scale_x, scale_y = preprocess_image(
            cv_image,
            input_size=(self.input_width, self.input_height),
        )

        outputs = self.backend.infer(input_tensor)
        raw_detections = decode_yolo_output(
            outputs=outputs,
            conf_threshold=self.conf_threshold,
            scale_x=scale_x,
            scale_y=scale_y,
            num_classes=get_num_classes(),
        )

        return [
            Detection(
                class_name=get_class_name(int(det['class_id'])),
                score=float(det['score']),
                x1=float(det['x1']),
                y1=float(det['y1']),
                x2=float(det['x2']),
                y2=float(det['y2']),
            )
            for det in raw_detections
        ]
