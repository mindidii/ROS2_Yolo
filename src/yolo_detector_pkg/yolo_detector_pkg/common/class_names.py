CLASS_NAMES = [
    "person",
    "drone",
]


def get_class_name(class_id: int) -> str:
    if 0 <= class_id < len(CLASS_NAMES):
        return CLASS_NAMES[class_id]
    return f"class_{class_id}"


def get_num_classes() -> int:
    return len(CLASS_NAMES)
