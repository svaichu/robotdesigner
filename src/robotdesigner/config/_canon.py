"""Canonical JSON and hashing shared by every config object."""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any, Callable, Mapping, TypeVar

from .errors import ConfigError

SIGNIFICANT_DIGITS = 12
HASH_DISPLAY_LENGTH = 12

T = TypeVar("T")


def round_float(x: float) -> float:
    """Round to 12 significant digits; ``-0.0`` becomes ``0.0``."""
    if x == 0.0:
        return 0.0
    return float(f"{x:.{SIGNIFICANT_DIGITS - 1}e}")


def canonicalize(obj: Any) -> Any:
    """Plain JSON-able structure with rounded floats and tuples turned into lists."""
    if isinstance(obj, bool) or obj is None or isinstance(obj, (str, int)):
        return obj
    if isinstance(obj, float):
        return round_float(obj)
    if isinstance(obj, Mapping):
        return {str(k): canonicalize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [canonicalize(v) for v in obj]
    raise TypeError(f"cannot canonicalize {type(obj).__name__}")


def canonical_json(obj: Any) -> str:
    return json.dumps(canonicalize(obj), sort_keys=True, separators=(",", ":"), allow_nan=False)


def sha256_hex(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


# --- field coercion and validation -------------------------------------------------


def as_float(x: Any, path: str) -> float:
    if isinstance(x, (bool, str, bytes)) or not hasattr(x, "__float__"):
        raise ConfigError(path, f"expected a number, got {type(x).__name__}")
    try:
        v = float(x)
    except (TypeError, ValueError):
        raise ConfigError(path, f"expected a number, got {x!r}") from None
    if not math.isfinite(v):
        raise ConfigError(path, "must be finite")
    return v


def as_vector(x: Any, size: int, path: str) -> tuple[float, ...]:
    try:
        items = tuple(x)
    except TypeError:
        raise ConfigError(path, f"expected a sequence of {size} numbers") from None
    if len(items) != size:
        raise ConfigError(path, f"expected {size} numbers, got {len(items)}")
    return tuple(as_float(v, f"{path}[{i}]") for i, v in enumerate(items))


def check_keys(d: Any, required: set[str], path: str, optional: set[str] = frozenset()) -> None:
    """Reject non-mappings, missing keys and unknown keys."""
    if not isinstance(d, Mapping):
        raise ConfigError(path, f"expected a mapping, got {type(d).__name__}")
    missing = sorted(required - set(d))
    if missing:
        raise ConfigError(path, f"missing keys: {', '.join(missing)}")
    unknown = sorted(set(d) - required - optional)
    if unknown:
        raise ConfigError(path, f"unknown keys: {', '.join(unknown)}")


def within(prefix: str, fn: Callable[..., T], *args: Any, **kwargs: Any) -> T:
    """Call ``fn`` and prefix the path of any ``ConfigError`` it raises."""
    try:
        return fn(*args, **kwargs)
    except ConfigError as e:
        raise e.prefixed(prefix) from None
