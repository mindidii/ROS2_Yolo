import onnxruntime as ort

# 모델을 어떤 프레임워크에서도 읽을 수 있도록 표준화한 저장 형식  
class OnnxObjectDetector:
    def __init__(self, model_path: str):
        self.session = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"]
        )
        # 모델이 입력받는 텐서의 이름을 가져옴, [0] 인 이유는 yolo 입력이 이미지 하나라서
        self.input_name = self.session.get_inputs()[0].name
        # 모델이 출력하는 텐서
        self.output_names = [output.name for output in self.session.get_outputs()]

    def infer(self, input_tensor):
        return self.session.run(self.output_names, {self.input_name: input_tensor})