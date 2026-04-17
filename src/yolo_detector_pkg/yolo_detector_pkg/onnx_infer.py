import onnxruntime as ort


class OnnxObjectDetector:
    def __init__(self, model_path: str):
        self.session = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"]
        )
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]

    def infer(self, input_tensor):
        return self.session.run(self.output_names, {self.input_name: input_tensor})