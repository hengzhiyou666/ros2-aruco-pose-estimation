#!/usr/bin/env python3

"""Image geometry helpers used by the ArUco ROS node."""

import numpy as np


def extract_nv12_y_plane(data, width: int, height: int, step: int) -> np.ndarray:
    """Return a tightly packed copy of the Y plane from an NV12 image buffer."""

    if width <= 0 or height <= 0:
        raise ValueError(f"Invalid NV12 image size: {width}x{height}")
    if step < width:
        raise ValueError(
            f"Invalid NV12 row step {step}: it is smaller than image width {width}"
        )

    raw = np.frombuffer(data, dtype=np.uint8)
    required_y_bytes = step * height
    if raw.size < required_y_bytes:
        raise ValueError(
            "NV12 buffer is too short for its Y plane: "
            f"got {raw.size} bytes, need at least {required_y_bytes}"
        )

    # Respect row padding described by step, but do not copy the interleaved UV plane.
    return raw[:required_y_bytes].reshape(height, step)[:, :width].copy()


def scale_camera_matrix(
    camera_matrix: np.ndarray,
    calibration_width: int,
    calibration_height: int,
    image_width: int,
    image_height: int,
) -> np.ndarray:
    """Scale camera intrinsics from calibration pixels to processing pixels."""

    if calibration_width <= 0 or calibration_height <= 0:
        raise ValueError(
            "CameraInfo must contain a positive calibration width and height"
        )
    if image_width <= 0 or image_height <= 0:
        raise ValueError(f"Invalid processing image size: {image_width}x{image_height}")

    scaled = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3).copy()
    scale_x = image_width / calibration_width
    scale_y = image_height / calibration_height

    # Pixel scaling is K' = diag(scale_x, scale_y, 1) * K.
    scaled[0, :] *= scale_x
    scaled[1, :] *= scale_y
    return scaled
