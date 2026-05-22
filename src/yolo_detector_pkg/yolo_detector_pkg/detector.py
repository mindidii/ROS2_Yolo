import os

from yolo_detector_pkg.class_names import get_class_name
from yolo_detector_pkg.class_names import get_num_classes
from yolo_detector_pkg.models import Detection
from yolo_detector_pkg.postprocess import decode_yolo_output
from yolo_detector_pkg.preprocess import preprocess_image

# 전체 파이프라인의 Wrapper 
class YoloDetector:
    def __init__(self, model_path: str, input_width: int, input_height: int, conf_threshold: float):
        self.model_path = model_path
        self.input_width = input_width
        self.input_height = input_height
        self.conf_threshold = conf_threshold
        self.backend = None

    # 실제 모델 파일 로드 
    def load(self):
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f'Model not found: {self.model_path}')

        model_ext = os.path.splitext(self.model_path)[1].lower()
        if model_ext == '.engine':
            from yolo_detector_pkg.tensorrt_infer import TensorRtObjectDetector

            self.backend = TensorRtObjectDetector(self.model_path)
            return

        if model_ext == '.onnx':
            from yolo_detector_pkg.onnx_infer import OnnxObjectDetector

            # onnxruntime 세션을 생성한 객체, 모델의 가중치를 들고 있는 주체
            self.backend = OnnxObjectDetector(self.model_path)
            return

        raise RuntimeError(f'Unsupported model format: {self.model_path}')
    # 모델 로드 여부 확인 
    def is_loaded(self) -> bool:
        return self.backend is not None

    def get_execution_providers(self) -> list[str]:
        if self.backend is None:
            return []
        return list(getattr(self.backend, 'providers', []))
    
    # 임계값 변경 
    def set_conf_threshold(self, conf_threshold: float):
        self.conf_threshold = conf_threshold
    
    # 이미지 입력 -> 추론 -> 객체 리스트 반환
    def detect(self, cv_image) -> list[Detection]:
        if self.backend is None:
            raise RuntimeError('Detector backend is not loaded')

        input_tensor, scale_x, scale_y = preprocess_image(
            cv_image,
            input_size=(self.input_width, self.input_height),
        )
        # 모델 추론 (backend=가중치를 들고 있는 주체, infer=입력 텐서를 모델에 통과해서 결과 숫자 배열을 반환)
        outputs = self.backend.infer(input_tensor)
        # 원본 이미지 좌표로 디코딩
        raw_detections = decode_yolo_output(
            outputs=outputs,
            conf_threshold=self.conf_threshold,
            scale_x=scale_x,
            scale_y=scale_y,
            num_classes=get_num_classes(),
        )
        # Detection 객체 리스트 반환
        return [
            Detection(
                class_id=int(det['class_id']),
                class_name=get_class_name(int(det['class_id'])),
                score=float(det['score']),
                x1=float(det['x1']),
                y1=float(det['y1']),
                x2=float(det['x2']),
                y2=float(det['y2']),
            )
            for det in raw_detections
        ]
