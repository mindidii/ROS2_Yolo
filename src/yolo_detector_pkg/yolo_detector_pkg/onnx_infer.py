import onnxruntime as ort


GPU_PROVIDERS = [
    "TensorrtExecutionProvider",
    "CUDAExecutionProvider",
]

CPU_PROVIDER = "CPUExecutionProvider"


def _select_providers():
    available = ort.get_available_providers()
    providers = [provider for provider in GPU_PROVIDERS if provider in available]

    if providers:
        return providers

    if CPU_PROVIDER in available:
        return [CPU_PROVIDER]

    raise RuntimeError(
        "No supported ONNX Runtime execution provider is available. "
        f"Available providers: {available}. "
        "Install an ONNX Runtime build with TensorrtExecutionProvider, "
        "CUDAExecutionProvider, or CPUExecutionProvider support."
    )


# 모델을 어떤 프레임워크에서도 읽을 수 있도록 표준화한 저장 형식  
class OnnxObjectDetector:
    def __init__(self, model_path: str):
        session_options = ort.SessionOptions()
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        self.session = ort.InferenceSession(
            model_path,
            sess_options=session_options,
            providers=_select_providers(),
        )
        self.providers = self.session.get_providers()
        # 모델이 입력받는 텐서의 이름을 가져옴, [0] 인 이유는 yolo 입력이 이미지 하나라서
        self.input_name = self.session.get_inputs()[0].name
        # 모델이 출력하는 텐서
        self.output_names = [output.name for output in self.session.get_outputs()]

    def infer(self, input_tensor):
        return self.session.run(self.output_names, {self.input_name: input_tensor})
