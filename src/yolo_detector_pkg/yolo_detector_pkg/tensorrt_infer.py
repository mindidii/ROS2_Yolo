import atexit
from dataclasses import dataclass

import numpy as np
import tensorrt as trt
from cuda.bindings import runtime as cudart


TRT_LOGGER = trt.Logger(trt.Logger.WARNING)


def _cuda_check(result):
    if not isinstance(result, tuple):
        if result != cudart.cudaError_t.cudaSuccess:
            raise RuntimeError(f"CUDA call failed with code {result}")
        return None

    status, *rest = result
    if status != cudart.cudaError_t.cudaSuccess:
        raise RuntimeError(f"CUDA call failed with code {status}")

    if not rest:
        return None
    if len(rest) == 1:
        return rest[0]
    return tuple(rest)


@dataclass
class _TensorBinding:
    name: str
    dtype: np.dtype
    shape: tuple[int, ...]
    size_bytes: int
    device_ptr: int


class TensorRtObjectDetector:
    def __init__(self, engine_path: str):
        self.providers = ["TensorRT"]
        self._runtime = trt.Runtime(TRT_LOGGER)

        with open(engine_path, "rb") as f:
            engine_data = f.read()

        self.engine = self._runtime.deserialize_cuda_engine(engine_data)
        if self.engine is None:
            raise RuntimeError(f"Failed to deserialize TensorRT engine: {engine_path}")

        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("Failed to create TensorRT execution context")

        self.stream = _cuda_check(cudart.cudaStreamCreate())
        self.bindings: dict[str, _TensorBinding] = {}
        self.host_outputs: dict[str, np.ndarray] = {}

        self.input_names = []
        self.output_names = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)

        if len(self.input_names) != 1:
            raise RuntimeError(
                f"Expected exactly one TensorRT input, found {len(self.input_names)}"
            )

        self.input_name = self.input_names[0]
        atexit.register(self.close)

    def close(self):
        for binding in self.bindings.values():
            try:
                _cuda_check(cudart.cudaFree(binding.device_ptr))
            except Exception:
                pass
        self.bindings.clear()
        self.host_outputs.clear()

        if getattr(self, "stream", None) is not None:
            try:
                _cuda_check(cudart.cudaStreamDestroy(self.stream))
            except Exception:
                pass
            self.stream = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def _ensure_binding(self, name: str, shape: tuple[int, ...]):
        dtype = np.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))
        size_bytes = int(np.prod(shape)) * dtype.itemsize
        existing = self.bindings.get(name)

        if existing is not None and (
            existing.shape != shape or existing.dtype != dtype or existing.size_bytes != size_bytes
        ):
            _cuda_check(cudart.cudaFree(existing.device_ptr))
            existing = None

        if existing is None:
            device_ptr = _cuda_check(cudart.cudaMalloc(size_bytes))
            existing = _TensorBinding(
                name=name,
                dtype=dtype,
                shape=shape,
                size_bytes=size_bytes,
                device_ptr=int(device_ptr),
            )
            self.bindings[name] = existing
        else:
            existing.shape = shape
            existing.size_bytes = size_bytes

        self.context.set_tensor_address(name, existing.device_ptr)
        return existing

    def infer(self, input_tensor):
        input_tensor = np.ascontiguousarray(input_tensor.astype(np.float32, copy=False))
        input_shape = tuple(int(v) for v in input_tensor.shape)

        if not self.context.set_input_shape(self.input_name, input_shape):
            raise RuntimeError(f"Failed to set TensorRT input shape: {input_shape}")

        input_binding = self._ensure_binding(self.input_name, input_shape)
        _cuda_check(
            cudart.cudaMemcpyAsync(
                input_binding.device_ptr,
                input_tensor.ctypes.data,
                input_binding.size_bytes,
                cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
                self.stream,
            )
        )

        outputs = []
        for name in self.output_names:
            output_shape = tuple(int(v) for v in self.context.get_tensor_shape(name))
            if any(dim < 0 for dim in output_shape):
                raise RuntimeError(f"TensorRT output shape is not fully specified: {name}")

            output_binding = self._ensure_binding(name, output_shape)
            host_output = np.empty(output_shape, dtype=output_binding.dtype)
            self.host_outputs[name] = host_output
            outputs.append(host_output)

        if not self.context.execute_async_v3(self.stream):
            raise RuntimeError("TensorRT execution failed")

        for name in self.output_names:
            output_binding = self.bindings[name]
            host_output = self.host_outputs[name]
            _cuda_check(
                cudart.cudaMemcpyAsync(
                    host_output.ctypes.data,
                    output_binding.device_ptr,
                    output_binding.size_bytes,
                    cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
                    self.stream,
                )
            )

        _cuda_check(cudart.cudaStreamSynchronize(self.stream))
        return [self.host_outputs[name].copy() for name in self.output_names]
