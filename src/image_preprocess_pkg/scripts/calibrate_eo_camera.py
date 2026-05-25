#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import numpy as np


def collect_calibration_points(image_paths, pattern_size, square_size):
    object_template = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    object_template[:, :2] = (
        np.mgrid[0:pattern_size[0], 0:pattern_size[1]]
        .T.reshape(-1, 2)
        .astype(np.float32)
    )
    object_template *= float(square_size)

    object_points = []
    image_points = []
    image_size = None
    used_images = []

    for path in image_paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = gray.shape[::-1]
        elif image_size != gray.shape[::-1]:
            print(f"Skipping {path}: size {gray.shape[::-1]} != {image_size}")
            continue

        found, corners = cv2.findChessboardCornersSB(gray, pattern_size)
        if not found:
            print(f"Skipping {path}: chessboard not found")
            continue

        object_points.append(object_template.copy())
        image_points.append(corners.astype(np.float32))
        used_images.append(path)

    return object_points, image_points, image_size, used_images


def format_dist(dist_coeffs):
    values = dist_coeffs.reshape(-1).tolist()
    while len(values) < 5:
        values.append(0.0)
    return values[:5]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Calibrate EO camera from saved 1280x720 chessboard images.")
    parser.add_argument("--image-dir", default="/tmp/eo_calibration_images")
    parser.add_argument("--glob", default="eo_calib_*.png")
    parser.add_argument("--cols", type=int, default=8, help="Inner chessboard corners per row")
    parser.add_argument("--rows", type=int, default=5, help="Inner chessboard corners per column")
    parser.add_argument("--square-size", type=float, default=1.0)
    parser.add_argument("--min-images", type=int, default=12)
    return parser.parse_args()


def main():
    args = parse_args()
    image_dir = Path(args.image_dir)
    image_paths = sorted(
        p for p in image_dir.glob(args.glob)
        if not p.name.endswith("_corners.png")
    )
    pattern_size = (args.cols, args.rows)

    object_points, image_points, image_size, used_images = collect_calibration_points(
        image_paths,
        pattern_size,
        args.square_size)

    if image_size is None:
        raise RuntimeError(f"No readable calibration images found in {image_dir}")

    if len(used_images) < args.min_images:
        raise RuntimeError(
            f"Need at least {args.min_images} valid images, found {len(used_images)}")

    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None)

    dist = format_dist(dist_coeffs)
    print(f"Used images: {len(used_images)}")
    print(f"Image size: {image_size[0]}x{image_size[1]}")
    print(f"RMS reprojection error: {rms:.6f}")
    print()
    print("Paste into src/sentinel_bringup/config/image_preprocess.yaml:")
    print(f"    eo_width: {image_size[0]}")
    print(f"    eo_height: {image_size[1]}")
    print(f"    calibration_width: {image_size[0]}")
    print(f"    calibration_height: {image_size[1]}")
    print(f"    camera_fx: {camera_matrix[0, 0]:.6f}")
    print(f"    camera_fy: {camera_matrix[1, 1]:.6f}")
    print(f"    camera_cx: {camera_matrix[0, 2]:.6f}")
    print(f"    camera_cy: {camera_matrix[1, 2]:.6f}")
    print(
        "    dist_coeffs: "
        f"[{dist[0]:.6f}, {dist[1]:.6f}, {dist[2]:.6f}, {dist[3]:.6f}, {dist[4]:.6f}]")


if __name__ == "__main__":
    main()
