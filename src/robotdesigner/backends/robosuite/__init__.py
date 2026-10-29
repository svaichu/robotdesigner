"""robosuite backend (work in progress).

Only the compatibility layer exists so far: :func:`apply_compat` lets robosuite 1.5.x run on current MuJoCo.
Importing this package does not import robosuite or mujoco.
"""

from ._compat import CompatReport, apply_compat

__all__ = ["CompatReport", "apply_compat"]
