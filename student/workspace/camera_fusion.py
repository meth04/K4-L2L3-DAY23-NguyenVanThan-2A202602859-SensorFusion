"""Camera field-of-view checks and pinhole measurement modeling.

Part G supplies visibility, projection, and pixel covariance (docs/HUONG_DAN_KY_THUAT.md §2).
The platform differentiates projection using a chain-rule Jacobian.
"""

from __future__ import annotations

from typing import Any
from typing import Sequence

import numpy as np

from fusion_lab.workspace_support import get_tracking_params

Matrix = np.matrix | np.ndarray

# vi: from fusion_lab.workspace_support import get_tracking_params

# Depth at or below this value cannot be projected (matches the platform guard).
_MIN_CAMERA_DEPTH = 1e-6


def _position_in_sensor(x: Matrix, sensor: Any) -> np.ndarray:
    """Rotate/translate a vehicle-frame position into the sensor frame.

    Args:
        x: State vector (6x1) with position in vehicle frame.
        sensor: Adapter exposing ``veh_to_sens`` (4x4 sensor<-vehicle transform).

    Returns:
        Sensor-frame position as a length-3 ``ndarray``.
    """
    transform = np.asarray(sensor.veh_to_sens, dtype=float)
    position = np.asarray(x, dtype=float).reshape(-1)[:3]
    return transform[:3, :3] @ position + transform[:3, 3]


def is_in_field_of_view(x: Matrix, sensor: Any) -> bool:
    """Return True if state x is visible within the sensor horizontal field of view.

    Args:
        x: State vector (6x1) with position in vehicle frame.
        sensor: Lidar or camera adapter with ``veh_to_sens`` and ``fov``
            (radians).

    Returns:
        True if sensor coordinates are finite and the horizontal angle is within
        ``sensor.fov``. A camera additionally requires depth > 1e-6.
    """
    # vi: TODO Part G — p_s = R @ p + t; loại tọa độ không hữu hạn.
    # vi: Camera cần x_s > 1e-6; FOV từ nội tại và bề rộng ảnh.
    # vi: Với cả lidar/camera: kiểm tra atan2(y_s, x_s) nằm trong sensor.fov.
    position = _position_in_sensor(x, sensor)
    if not np.isfinite(position).all():
        return False
    if sensor.name == "camera" and position[0] <= _MIN_CAMERA_DEPTH:
        return False
    angle = np.arctan2(position[1], position[0])
    low, high = sensor.fov
    return bool(low <= angle <= high)


def camera_measurement_prediction(x: Matrix, sensor: Any) -> Matrix:
    """Predict image-plane measurement h(x) using the pinhole camera model.

    Args:
        x: State vector.
        sensor: Camera with intrinsics ``f_i, f_j, c_i, c_j``.

    Returns:
        2x1 predicted pixel coordinates as ``np.matrix``.

    Raises:
        ValueError: With coordinate context if sensor coordinates are nonfinite
            or depth is at most 1e-6.
    """
    # vi: TODO Part G — tính p_s = R @ p + t; trước phép chia kiểm tra hữu hạn
    # vi: và x_s > 1e-6, ngược lại raise ValueError có tọa độ.
    # vi: u = c_i - f_i * y_s/x_s; v = c_j - f_j * z_s/x_s.
    position = _position_in_sensor(x, sensor)
    if not np.isfinite(position).all() or position[0] <= _MIN_CAMERA_DEPTH:
        raise ValueError(
            "Camera projection needs finite coordinates and positive depth "
            f"> {_MIN_CAMERA_DEPTH:g}; sensor position={position.tolist()}"
        )
    depth, left, up = position
    u = sensor.c_i - sensor.f_i * left / depth
    v = sensor.c_j - sensor.f_j * up / depth
    return np.asmatrix([[u], [v]])


def build_camera_measurement(z: Sequence[float], sensor: Any) -> dict[str, Any]:
    """Build camera measurement vector z and covariance R from pixel coordinates.

    Args:
        z: Sequence ``[u, v]`` pixel coordinates.
        sensor: Camera sensor object.

    Returns:
        Dict with keys ``z``, ``R``, ``sensor``.
    """
    # vi: TODO Part G — z mat 2x1; R diag sigma_cam_i^2, sigma_cam_j^2 từ params.
    params = get_tracking_params()
    z_mat = np.asmatrix(np.asarray(z, dtype=float).reshape(2, 1))
    covariance = np.asmatrix(
        np.diag([params.sigma_cam_i**2, params.sigma_cam_j**2])
    )
    return {"z": z_mat, "R": covariance, "sensor": sensor}
