"""EquirectProjector: one cached fisheye->equirect remap plus angle helpers in ROS sign.

Sign convention: ``pixel_to_heading_elevation`` returns heading in ROS/REP-103 (+left) by negating the
image-native (+right) heading from ``fisheye_utils``. Elevation is +up.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

from . import fisheye_utils

RAW_WIDTH = 1400
RAW_HEIGHT = 1050


class EquirectProjector:
    def __init__(
        self,
        width: int = 800,
        height: int = 720,
        h_fov_deg: float = 180.0,
        v_fov_deg: float = 180.0,
        camera_params_path: Optional[str] = None,
        raw_width: int = RAW_WIDTH,
        raw_height: int = RAW_HEIGHT,
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.h_fov_deg = float(h_fov_deg)
        self.v_fov_deg = float(v_fov_deg)
        self.raw_width = raw_width
        self.raw_height = raw_height
        self.model = fisheye_utils.create_fisheye_model_from_params(camera_params_path, raw_width, raw_height)
        self._remap = fisheye_utils.FisheyeToEquirectangular(
            self.width, self.height, self.h_fov_deg, self.v_fov_deg, self.model
        )

    def project(self, img: np.ndarray) -> np.ndarray:
        """Remap a raw fisheye frame (BGR or RGB, HxWxC) to the equirect panorama. Channel order is preserved."""
        if img.shape[1] != self.raw_width or img.shape[0] != self.raw_height:
            # Maps were built for the nominal raw size; scale the maps rather than the image.
            sx = img.shape[1] / self.raw_width
            sy = img.shape[0] / self.raw_height
            import cv2

            return cv2.remap(
                img,
                self._remap.map_x * sx,
                self._remap.map_y * sy,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0),
            )
        return self._remap.project(img)

    def pixel_to_heading_elevation(self, u: float, v: float) -> Tuple[float, float]:
        """Equirect pixel -> (heading_deg ROS +left, elevation_deg +up)."""
        elevation, heading_right = fisheye_utils.equirectangular_pixel_to_elevation_heading(
            float(u), float(v), self.width, self.height, self.h_fov_deg, self.v_fov_deg
        )
        return -float(heading_right), float(elevation)

    @staticmethod
    def floor_distance(elevation_deg: float, camera_height_m: float = 0.2) -> Optional[float]:
        """Ground-contact range from the elevation of a bbox bottom edge. None unless the edge is below -3 deg."""
        if elevation_deg is None or elevation_deg >= -3.0:
            return None
        return camera_height_m / math.tan(math.radians(-elevation_deg))
