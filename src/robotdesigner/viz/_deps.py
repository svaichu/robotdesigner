"""Optional-dependency helpers. No third-party import at module level."""

from __future__ import annotations

import importlib.util

INSTALL_HINT = "pip install 'robotdesigner[viz]'"


def have(module: str) -> bool:
    return importlib.util.find_spec(module) is not None


def need(module: str, what: str) -> None:
    if not have(module):
        raise ImportError(f"{what} needs the optional dependency {module!r}; install it with {INSTALL_HINT}")
