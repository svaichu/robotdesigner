"""``Clip``: a stack of RGB frames with a frame rate. Saves GIF or MP4 and displays inline in notebooks."""

from __future__ import annotations

import base64
import html
import io
import tempfile
import warnings
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ._deps import have, need

FORMATS = ("mp4", "gif")
_GIF_MIN_MS = 20  # most browsers play shorter GIF delays at 100 ms
_EMBED_WARN_MB = 25.0


def pad_even(frames: np.ndarray) -> np.ndarray:
    """Repeat the last row / column so height and width are even (yuv420p needs even sizes)."""
    h, w = frames.shape[1:3]
    if h % 2 == 0 and w % 2 == 0:
        return frames
    return np.pad(frames, ((0, 0), (0, h % 2), (0, w % 2), (0, 0)), mode="edge")


def gif_delay_ms(fps: float) -> int:
    """Frame delay stored in a GIF. GIF counts in 10 ms steps, so only some rates are exact."""
    wanted = 1000.0 / fps
    delay = max(int(round(wanted / 10.0)) * 10, _GIF_MIN_MS)
    if abs(delay - wanted) > 0.5:
        warnings.warn(
            f"GIF stores frame delays in 10 ms steps (minimum {_GIF_MIN_MS} ms): {fps:g} fps will play at "
            f"{1000.0 / delay:.4g} fps. Use 50, 25, 20 or 10 fps for an exact rate, or save an mp4.",
            UserWarning,
            stacklevel=3,
        )
    return delay


def _encode_gif(frames: np.ndarray, fps: float) -> bytes:
    need("PIL", "GIF output")
    from PIL import Image

    delay = gif_delay_ms(fps)
    images = [Image.fromarray(f) for f in frames]
    buf = io.BytesIO()
    images[0].save(buf, format="GIF", save_all=True, append_images=images[1:], duration=delay, loop=0)
    return buf.getvalue()


def _write_mp4(frames: np.ndarray, fps: float, path: Path) -> None:
    need("imageio", "MP4 output")
    need("imageio_ffmpeg", "MP4 output")
    import imageio.v2 as iio

    iio.mimsave(
        path,
        list(pad_even(frames)),
        fps=fps,
        codec="libx264",
        pixelformat="yuv420p",
        macro_block_size=1,  # sizes are already even; the default would rescale to a multiple of 16
        output_params=["-crf", "23", "-preset", "medium", "-movflags", "+faststart"],
    )


@dataclass(frozen=True, eq=False, repr=False)
class Clip:
    """``frames`` is ``uint8 (T, H, W, 3)``; ``fps`` is the playback rate. Treat both as immutable."""

    frames: np.ndarray
    fps: float

    def __post_init__(self):
        f = np.asarray(self.frames)
        if f.dtype != np.uint8 or f.ndim != 4 or f.shape[-1] != 3:
            raise ValueError(f"frames must be uint8 with shape (T, H, W, 3), got {f.dtype} {f.shape}")
        if min(f.shape[:3]) < 1:
            raise ValueError(f"a clip needs at least one non-empty frame, got shape {f.shape}")
        fps = float(self.fps)
        if not (fps > 0.0 and np.isfinite(fps)):
            raise ValueError(f"fps must be a positive number, got {self.fps!r}")
        object.__setattr__(self, "frames", np.ascontiguousarray(f))
        object.__setattr__(self, "fps", fps)

    def __len__(self) -> int:
        return int(self.frames.shape[0])

    @property
    def size(self) -> tuple[int, int]:
        """``(height, width)`` of one frame."""
        return int(self.frames.shape[1]), int(self.frames.shape[2])

    @property
    def duration(self) -> float:
        """Playing time in seconds."""
        return len(self) / self.fps

    def __repr__(self) -> str:
        h, w = self.size
        return f"Clip({len(self)} frames, {h}x{w}, {self.fps:g} fps, {self.duration:.2f} s)"

    def with_fps(self, fps: float) -> "Clip":
        return Clip(self.frames, fps)

    # --- files -------------------------------------------------------------------------------------

    def to_bytes(self, fmt: str = "mp4") -> bytes:
        fmt = _check_format(fmt)
        if fmt == "gif":
            return _encode_gif(self.frames, self.fps)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "clip.mp4"
            _write_mp4(self.frames, self.fps, path)
            return path.read_bytes()

    def save(self, path, fmt: str | None = None) -> Path:
        """Write the clip. The format is ``fmt`` or the suffix of ``path`` (``.mp4`` or ``.gif``)."""
        path = Path(path)
        fmt = _check_format(fmt or path.suffix.lstrip(".").lower(), what=f"{path.name!r}")
        path.parent.mkdir(parents=True, exist_ok=True)
        if fmt == "gif":
            path.write_bytes(_encode_gif(self.frames, self.fps))
        else:
            _write_mp4(self.frames, self.fps, path)
        return path

    # --- notebooks ---------------------------------------------------------------------------------

    def _html(self, fmt: str | None) -> str:
        note = ""
        if fmt is None:  # automatic: MP4 when it can be written, else GIF
            if have("imageio") and have("imageio_ffmpeg"):
                fmt = "mp4"
            else:
                need("PIL", "inline display (neither MP4 nor GIF can be written)")
                fmt = "gif"
                note = "MP4 needs imageio-ffmpeg (pip install 'robotdesigner[viz]'); showing a GIF instead."
        data = self.to_bytes(fmt)
        if len(data) / 1e6 > _EMBED_WARN_MB:
            warnings.warn(
                f"embedding a {len(data) / 1e6:.0f} MB {fmt} in the notebook will make it slow; "
                "save it and open the file instead, or record fewer frames (every=...)",
                UserWarning,
                stacklevel=3,
            )
        b64 = base64.b64encode(data).decode("ascii")
        if fmt == "mp4":
            media = (
                '<video controls loop autoplay muted playsinline style="max-width:100%" '
                f'src="data:video/mp4;base64,{b64}"></video>'
            )
        else:
            media = f'<img style="max-width:100%" src="data:image/gif;base64,{b64}"/>'
        if note:
            media += f'<div style="font:12px sans-serif;color:#666">{html.escape(note)}</div>'
        return media

    def _repr_html_(self) -> str:
        """Shown by Jupyter / VS Code when a ``Clip`` is the last expression of a cell."""
        try:
            return self._html(None)
        except ImportError as e:
            return f"<pre>{html.escape(repr(self))}\n{html.escape(str(e))}</pre>"

    def show(self, fmt: str | None = None) -> None:
        """Display inline now. ``fmt`` is ``"mp4"`` or ``"gif"``; ``None`` picks MP4 when it is available."""
        from IPython.display import HTML, display

        display(HTML(self._html(None if fmt is None else _check_format(fmt))))


def _check_format(fmt: str, what: str = "format") -> str:
    if fmt not in FORMATS:
        raise ValueError(f"cannot write {what}: expected one of {list(FORMATS)}, got {fmt!r}")
    return fmt
