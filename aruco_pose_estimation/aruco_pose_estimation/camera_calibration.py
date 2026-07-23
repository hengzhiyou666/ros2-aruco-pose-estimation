#!/usr/bin/env python3

"""Load camera intrinsics from the VITA multi-camera calibration YAML."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import yaml


SUPPORTED_DISTORTION_LENGTHS = {4, 5, 8, 12, 14}


@dataclass(frozen=True)
class CameraCalibration:
    """Validated pinhole calibration for one named camera."""

    label: str
    width: int
    height: int
    camera_matrix: tuple[float, ...]
    distortion: tuple[float, ...]
    distortion_model: str


def _mapping(value: Any, field: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{field} must be a YAML mapping")
    return value


def _positive_int(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{field} must be a positive integer")
    return value


def _finite_numbers(value: Any, field: str) -> list[float]:
    if not isinstance(value, list):
        raise ValueError(f"{field} must be a YAML list")

    numbers = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(f"{field}[{index}] must be numeric")
        number = float(item)
        if not math.isfinite(number):
            raise ValueError(f"{field}[{index}] must be finite")
        numbers.append(number)
    return numbers


def _matrix_data(
    matrix: Any,
    field: str,
    allowed_lengths: set[int],
) -> list[float]:
    matrix = _mapping(matrix, field)
    rows = _positive_int(matrix.get("rows"), f"{field}.rows")
    cols = _positive_int(matrix.get("cols"), f"{field}.cols")
    data = _finite_numbers(matrix.get("data"), f"{field}.data")

    if rows * cols != len(data):
        raise ValueError(
            f"{field} declares {rows}x{cols} values but contains {len(data)}"
        )
    if len(data) not in allowed_lengths:
        expected = ", ".join(str(length) for length in sorted(allowed_lengths))
        raise ValueError(
            f"{field}.data has {len(data)} values; expected one of: {expected}"
        )
    return data


def load_vita_camera_calibration(
    calibration_file: str,
    camera_label: str,
) -> CameraCalibration:
    """Load one camera selected by label from a VITA calibration file."""

    if not calibration_file:
        raise ValueError("camera_calibration_file must not be empty")
    if not camera_label:
        raise ValueError("camera_calibration_label must not be empty")

    path = Path(calibration_file).expanduser()
    try:
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except OSError as error:
        raise ValueError(f"cannot read calibration file {path}: {error}") from error
    except yaml.YAMLError as error:
        raise ValueError(f"invalid YAML in calibration file {path}: {error}") from error

    document = _mapping(document, str(path))
    cameras = document.get("cameras")
    if not isinstance(cameras, list) or not cameras:
        raise ValueError(f"{path}: cameras must be a non-empty YAML list")

    matches = []
    for index, entry in enumerate(cameras):
        entry = _mapping(entry, f"{path}: cameras[{index}]")
        camera = _mapping(entry.get("camera"), f"{path}: cameras[{index}].camera")
        if camera.get("label") == camera_label:
            matches.append(camera)

    if not matches:
        raise ValueError(f"{path}: camera label {camera_label!r} was not found")
    if len(matches) > 1:
        raise ValueError(f"{path}: camera label {camera_label!r} is duplicated")

    camera = matches[0]
    if camera.get("type") != "pinhole":
        raise ValueError(
            f"{path}: camera {camera_label!r} must have type 'pinhole'"
        )

    width = _positive_int(
        camera.get("image_width"),
        f"{path}: camera {camera_label!r}.image_width",
    )
    height = _positive_int(
        camera.get("image_height"),
        f"{path}: camera {camera_label!r}.image_height",
    )

    intrinsics = _matrix_data(
        camera.get("intrinsics"),
        f"{path}: camera {camera_label!r}.intrinsics",
        {4},
    )
    fx, fy, cx, cy = intrinsics
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError(
            f"{path}: camera {camera_label!r} fx and fy must be positive"
        )

    distortion_section = _mapping(
        camera.get("distortion"),
        f"{path}: camera {camera_label!r}.distortion",
    )
    if distortion_section.get("type") != "radial_tangential":
        raise ValueError(
            f"{path}: camera {camera_label!r} distortion type must be "
            "'radial_tangential'"
        )
    distortion = _matrix_data(
        distortion_section.get("parameters"),
        f"{path}: camera {camera_label!r}.distortion.parameters",
        SUPPORTED_DISTORTION_LENGTHS,
    )

    camera_matrix = (
        fx,
        0.0,
        cx,
        0.0,
        fy,
        cy,
        0.0,
        0.0,
        1.0,
    )
    distortion_model = (
        "rational_polynomial" if len(distortion) >= 8 else "plumb_bob"
    )

    return CameraCalibration(
        label=camera_label,
        width=width,
        height=height,
        camera_matrix=camera_matrix,
        distortion=tuple(distortion),
        distortion_model=distortion_model,
    )
