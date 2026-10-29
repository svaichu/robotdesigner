"""Visualization: ``Clip`` (save GIF / MP4, show inline), frame recording, camera framing.

Importing this package imports numpy only. Pillow, imageio-ffmpeg and mujoco are loaded when first needed.
"""

from .camera import VIEWS, CameraSpec, auto_camera, workspace_points
from .clip import Clip
from .frames import annotate, tile, tile_clips
from .record import FrameRecorder, record_rollout

__all__ = [
    "VIEWS",
    "CameraSpec",
    "Clip",
    "FrameRecorder",
    "annotate",
    "auto_camera",
    "record_rollout",
    "tile",
    "tile_clips",
    "workspace_points",
]
