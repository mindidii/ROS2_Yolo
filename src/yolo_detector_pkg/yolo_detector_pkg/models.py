from dataclasses import dataclass


@dataclass
class Detection:
    class_name: str # 탐지된 객체의 클래스 이름 
    score: float    # 탐지 신뢰도 
    x1: float   # 탐지된 객체의 바운딩 박스 좌표 (왼쪽 상단)
    y1: float
    x2: float
    y2: float
