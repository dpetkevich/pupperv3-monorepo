"""Fisheye (double-sphere) camera model and equirectangular projection.

Copied from ``ros2_ws/src/hailo/hailo/fisheye_utils.py`` with the latent TypeError in
``convert_boxes_to_elevation_heading`` fixed (it now takes ``v_fov_deg``) and prints removed.

Sign convention (image-native): ``heading_deg`` is +right (u increasing), ``elevation_deg`` is +up.
"""
from __future__ import annotations

import os
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
import yaml

DEFAULT_CAMERA_PARAMS = os.path.join(os.path.dirname(__file__), "camera_params.yaml")


class CameraModel:
    def __init__(self, cx, cy, fx, fy, width=None, height=None):
        self.cx = cx
        self.cy = cy
        self.fx = fx
        self.fy = fy
        self.width = width
        self.height = height


class DoubleSphereModel(CameraModel):
    def __init__(self, cx, cy, fx, fy, xi, alpha, width=None, height=None):
        super().__init__(cx, cy, fx, fy, width, height)
        self.xi = xi
        self.alpha = alpha

    def project(self, x, y, z, eps=1e-9):
        r2 = x * x + y * y
        d1 = np.sqrt(r2 + z * z)
        k2 = self.xi * d1 + z
        d2 = np.sqrt(r2 + k2 * k2)
        denom_raw = self.alpha * d2 + (1.0 - self.alpha) * k2

        valid = denom_raw > 0
        denom = np.maximum(denom_raw, eps)

        mx = x / denom
        my = y / denom

        u = self.fx * mx + self.cx
        v = self.fy * my + self.cy
        return u, v, valid

    def unproject(self, u, v):
        mx = (u - self.cx) / self.fx
        my = (v - self.cy) / self.fy

        r2 = mx * mx + my * my
        mz = (1 - self.alpha * self.alpha * r2) / (
            self.alpha * np.sqrt(1 - (2 * self.alpha - 1) * r2) + 1 - self.alpha
        )
        scale = (mz * self.xi + np.sqrt(mz * mz + (1 - self.xi * self.xi) * r2)) / (mz * mz + r2)

        x = scale * mx
        y = scale * my
        z = scale * mz - self.xi

        norm = np.sqrt(x * x + y * y + z * z)
        return x / norm, y / norm, z / norm


class PinholeModel(CameraModel):
    def project(self, x, y, z, eps=1e-9):
        valid = z > eps
        z_safe = np.maximum(z, eps)
        u = self.fx * (x / z_safe) + self.cx
        v = self.fy * (y / z_safe) + self.cy
        if self.width is not None and self.height is not None:
            valid = valid & (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)
        return u, v, valid

    def unproject(self, u, v):
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        z = 1.0
        norm = np.sqrt(x * x + y * y + z * z)
        return x / norm, y / norm, z / norm


def create_equirectangular_rays(width: int, height: int, h_fov_deg: float, v_fov_deg: float):
    """Unit rays for every equirect pixel. Column 0 = -h_fov/2 (left), row 0 = -v_fov/2 (camera-up is -y)."""
    h_fov_rad = np.deg2rad(h_fov_deg)
    v_fov_rad = np.deg2rad(v_fov_deg)
    lon = (np.linspace(0, width - 1, width) / (width - 1)) * h_fov_rad - (h_fov_rad / 2)
    lat = (np.linspace(0, height - 1, height) / (height - 1)) * v_fov_rad - (v_fov_rad / 2)
    lon_grid, lat_grid = np.meshgrid(lon, lat)
    x = np.cos(lat_grid) * np.sin(lon_grid)
    y = np.sin(lat_grid)
    z = np.cos(lat_grid) * np.cos(lon_grid)
    return x, y, z


class FisheyeToEquirectangular:
    """Cached remap from a fisheye frame to an equirectangular panorama. Build once, call ``project`` per frame."""

    def __init__(self, out_width: int, out_height: int, h_fov_deg: float, v_fov_deg: float, fisheye_model: CameraModel):
        x, y, z = create_equirectangular_rays(out_width, out_height, h_fov_deg, v_fov_deg)
        u, v, self.valid = fisheye_model.project(x, y, z)
        self.map_x = u.astype(np.float32)
        self.map_y = v.astype(np.float32)

    def project(self, img: np.ndarray) -> np.ndarray:
        pano = cv2.remap(
            img,
            self.map_x,
            self.map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
        if pano.ndim == 3:
            pano[~self.valid] = (0, 0, 0)
        else:
            pano[~self.valid] = 0
        return pano


def load_camera_params(config_path: str | None = None) -> Dict[str, float]:
    """Load double-sphere parameters from YAML (defaults to the packaged camera_params.yaml)."""
    config_path = config_path or DEFAULT_CAMERA_PARAMS
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    params = config.get("camera_params", {})
    return {k: float(params[k]) for k in ("fx", "fy", "cx", "cy", "xi", "alpha")}


def create_fisheye_model_from_params(config_path: str | None, img_width: int, img_height: int) -> DoubleSphereModel:
    p = load_camera_params(config_path)
    return DoubleSphereModel(
        cx=p["cx"], cy=p["cy"], fx=p["fx"], fy=p["fy"], xi=p["xi"], alpha=p["alpha"], width=img_width, height=img_height
    )


def equirectangular_pixel_to_elevation_heading(u, v, width, height, h_fov_deg, v_fov_deg) -> Tuple[Any, Any]:
    """Equirect pixel -> (elevation_deg, heading_deg). Image-native sign: heading +right, elevation +up."""
    h_fov_rad = np.deg2rad(h_fov_deg)
    v_fov_rad = np.deg2rad(v_fov_deg)
    heading_rad = (u / (width - 1)) * h_fov_rad - (h_fov_rad / 2)
    elevation_rad = (1.0 - v / (height - 1)) * v_fov_rad - (v_fov_rad / 2)
    return np.rad2deg(elevation_rad), np.rad2deg(heading_rad)


def convert_boxes_to_elevation_heading(
    boxes: List[Any], equirect_width: int, equirect_height: int, h_fov_deg: float, v_fov_deg: float
) -> List[Dict[str, Any]]:
    """Box centroids -> elevation/heading dicts (image-native sign, heading +right).

    Fixed from the Hailo copy, which omitted ``v_fov_deg`` and raised TypeError.
    """
    out = []
    for box in boxes:
        cu = (box.x1 + box.x2) / 2.0
        cv_ = (box.y1 + box.y2) / 2.0
        elevation_deg, heading_deg = equirectangular_pixel_to_elevation_heading(
            cu, cv_, equirect_width, equirect_height, h_fov_deg, v_fov_deg
        )
        out.append(
            {
                "label": box.label,
                "centroid": (cu, cv_),
                "elevation_deg": float(elevation_deg),
                "heading_deg": float(heading_deg),
            }
        )
    return out
