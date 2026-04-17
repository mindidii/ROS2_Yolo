import cv2
import numpy as np


def preprocess_image(image_bgr, input_size=(640, 640)):
    """
    BGR OpenCV 이미지를 YOLO ONNX 입력 텐서로 변환
    반환:
      input_tensor: (1, 3, H, W) float32
      scale_x, scale_y: bbox 원복용 스케일
    """
    orig_h, orig_w = image_bgr.shape[:2]
    input_w, input_h = input_size

    resized = cv2.resize(image_bgr, (input_w, input_h))
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

    img = rgb.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))   # HWC -> CHW
    img = np.expand_dims(img, axis=0)    # CHW -> BCHW

    scale_x = orig_w / float(input_w)
    scale_y = orig_h / float(input_h)

    return img, scale_x, scale_y