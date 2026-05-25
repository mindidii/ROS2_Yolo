import cv2
import numpy as np


def preprocess_image(image_bgr, input_size=(640, 640)):
    """
    BGR OpenCV 이미지를 YOLO 입력 텐서로 변환합니다.
    원본 비율을 유지하는 letterbox를 사용해 16:9 EO 영상이 찌그러지지 않게 합니다.
    """
    orig_h, orig_w = image_bgr.shape[:2]
    input_w, input_h = input_size

    gain = min(input_w / float(orig_w), input_h / float(orig_h))
    resized_w = int(round(orig_w * gain))
    resized_h = int(round(orig_h * gain))
    pad_x = (input_w - resized_w) / 2.0
    pad_y = (input_h - resized_h) / 2.0

    resized = cv2.resize(
        image_bgr,
        (resized_w, resized_h),
        interpolation=cv2.INTER_LINEAR,
    )
    letterboxed = np.full((input_h, input_w, 3), 114, dtype=np.uint8)
    left = int(round(pad_x - 0.1))
    top = int(round(pad_y - 0.1))
    letterboxed[top:top + resized_h, left:left + resized_w] = resized

    rgb = cv2.cvtColor(letterboxed, cv2.COLOR_BGR2RGB)

    img = rgb.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))   # HWC -> CHW
    img = np.expand_dims(img, axis=0)    # CHW -> BCHW

    transform = {
        "mode": "letterbox",
        "gain": float(gain),
        "pad_x": float(left),
        "pad_y": float(top),
        "orig_w": int(orig_w),
        "orig_h": int(orig_h),
    }

    return img, transform
