"""Frame helpers: text labels and tiling. Pillow is imported only by ``annotate``."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from ._deps import need
from .clip import Clip


def _font(size: int):
    from PIL import ImageFont

    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Pillow < 10.1: fixed-size bitmap font
        return ImageFont.load_default()


def annotate(frame: np.ndarray, text: str, *, corner: str = "tl", size: int = 14) -> np.ndarray:
    """Draw white text on a translucent dark box in a corner (``tl``, ``tr``, ``bl``, ``br``)."""
    need("PIL", "annotate()")
    from PIL import Image, ImageDraw

    if corner not in ("tl", "tr", "bl", "br"):
        raise ValueError(f"corner must be one of tl, tr, bl, br; got {corner!r}")
    base = Image.fromarray(np.asarray(frame)).convert("RGBA")
    layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    font = _font(size)
    left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
    pad = 3
    bw, bh = right - left + 2 * pad, bottom - top + 2 * pad
    x = 0 if corner in ("tl", "bl") else base.width - bw
    y = 0 if corner in ("tl", "tr") else base.height - bh
    draw.rectangle([x, y, x + bw, y + bh], fill=(0, 0, 0, 170))
    draw.text((x + pad - left, y + pad - top), text, font=font, fill=(255, 255, 255, 255))
    return np.asarray(Image.alpha_composite(base, layer).convert("RGB"))


def _grid(n: int, ncols: int | None) -> tuple[int, int]:
    if n < 1:
        raise ValueError("nothing to tile")
    if ncols is None:
        ncols = math.ceil(math.sqrt(n))
    if ncols < 1:
        raise ValueError(f"ncols must be >= 1, got {ncols}")
    ncols = min(int(ncols), n)
    return math.ceil(n / ncols), ncols


def _tile_array(arr: np.ndarray, ncols: int | None, pad: int) -> np.ndarray:
    """``(N, T, H, W, 3) -> (T, rows*H + ..., cols*W + ..., 3)``; a border of ``pad`` pixels around every tile."""
    n, t, h, w, _ = arr.shape
    rows, cols = _grid(n, ncols)
    out = np.zeros((t, rows * h + (rows + 1) * pad, cols * w + (cols + 1) * pad, 3), dtype=np.uint8)
    for k in range(n):
        r, c = divmod(k, cols)
        y, x = pad + r * (h + pad), pad + c * (w + pad)
        out[:, y : y + h, x : x + w] = arr[k]
    return out


def tile(frames: Sequence[np.ndarray], ncols: int | None = None, pad: int = 2) -> np.ndarray:
    """Arrange equally sized ``(H, W, 3)`` frames in a grid (near-square unless ``ncols`` is given)."""
    arr = np.stack([np.asarray(f) for f in frames])
    if arr.dtype != np.uint8 or arr.ndim != 4 or arr.shape[-1] != 3:
        raise ValueError(f"frames must be equally sized uint8 (H, W, 3) images, got {arr.dtype} {arr.shape}")
    return _tile_array(arr[:, None], ncols, pad)[0]


def tile_clips(clips: Sequence[Clip], ncols: int | None = None, labels: Sequence[str] | None = None, pad: int = 2) -> Clip:
    """Play clips side by side. They need the same frame size and fps; the result is as long as the shortest."""
    clips = list(clips)
    if not clips:
        raise ValueError("nothing to tile")
    if len({c.size for c in clips}) != 1:
        raise ValueError(f"clips must have the same frame size, got {[c.size for c in clips]}")
    if len({c.fps for c in clips}) != 1:
        raise ValueError(f"clips must have the same fps, got {[c.fps for c in clips]}")
    if labels is not None and len(labels) != len(clips):
        raise ValueError(f"expected {len(clips)} labels, got {len(labels)}")
    t = min(len(c) for c in clips)
    stacks = []
    for k, c in enumerate(clips):
        f = c.frames[:t]
        if labels is not None:
            f = np.stack([annotate(x, labels[k]) for x in f])
        stacks.append(f)
    return Clip(_tile_array(np.stack(stacks), ncols, pad), clips[0].fps)
