from dataclasses import dataclass


@dataclass
class Detection:
    class_name: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float
