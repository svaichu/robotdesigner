"""Compatibility patches that let robosuite 1.5.x run on current MuJoCo.

robosuite 1.5.x declares ``mujoco>=3.3.0`` with no upper bound, but two of its code paths break on newer MuJoCo:

``joint-address-enum``
    ``MjModel.get_joint_qpos_addr`` and ``get_joint_qvel_addr`` test ``np.int32 in (mjJNT_HINGE, mjJNT_SLIDE)``.
    From MuJoCo 3.12 (pybind11 3.1 enum comparison) that is False, so every hinge or slide joint fails an assert
    during the first reset.

``mass-matrix``
    ``Controller.update`` calls ``mujoco.mj_fullM(model, dst, data.qM)``. MuJoCo 3.10 changed the signature to
    ``mj_fullM(model, data, dst)`` and 3.11 removed ``MjData.qM``.

Upstream reports: ARISE-Initiative/robosuite issue 831 (version pin) and pull request 832 (enum comparison).

:func:`apply_compat` is idempotent. It probes each problem (the joint-address check runs robosuite's own method,
the mass-matrix check reads robosuite's source and tries the old MuJoCo call) and patches only what fails, so it
does nothing on old MuJoCo and switches itself off once robosuite fixes the code. Each patch is narrow: the
joint-address methods are replaced by int-comparing versions, and the mass-matrix call is routed through a
stand-in that supplies the ``MjData`` the new signature needs.

The patches live in the current process. Call :func:`apply_compat` in every worker that starts a fresh interpreter
(``spawn`` or ``forkserver``); forked workers inherit them.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

__all__ = ["CompatReport", "apply_compat"]

_PROBE_XML = "<mujoco><worldbody><body><joint name='j' type='hinge'/><geom size='.1'/></body></worldbody></mujoco>"

_LOCK = threading.Lock()
_REPORT: CompatReport | None = None


@dataclass(frozen=True, slots=True)
class CompatReport:
    """What :func:`apply_compat` found: patches applied now, and checks that passed without a patch."""

    mujoco_version: str
    robosuite_version: str
    applied: tuple[str, ...]
    not_needed: tuple[str, ...]


class _LegacyMassMatrix:
    """Stand-in for the removed ``MjData.qM``. It only remembers which ``MjData`` it came from."""

    __slots__ = ("data",)

    def __init__(self, data):
        self.data = data


class _MujocoProxy:
    """Replaces the ``mujoco`` name inside robosuite's controller module. Forwards everything but ``mj_fullM``."""

    def __init__(self, real):
        object.__setattr__(self, "_real", real)

    def __getattr__(self, name):
        return getattr(self._real, name)

    def mj_fullM(self, model, dst, qm):
        if isinstance(qm, _LegacyMassMatrix):
            return self._real.mj_fullM(model, qm.data, dst)  # MuJoCo >= 3.10 signature
        return self._real.mj_fullM(model, dst, qm)  # old signature, untouched


def _joint_address_broken(mujoco, binding_utils) -> bool:
    model = binding_utils.MjModel(mujoco.MjModel.from_xml_string(_PROBE_XML))
    try:
        model.get_joint_qpos_addr("j")
        model.get_joint_qvel_addr("j")
    except AssertionError:
        return True
    return False


def _mass_matrix_broken(mujoco, controller_module) -> bool:
    """True when robosuite still reads ``data.qM`` and the installed MuJoCo no longer supports that call."""
    import ast
    import inspect
    import textwrap

    import numpy as np

    try:
        tree = ast.parse(textwrap.dedent(inspect.getsource(controller_module.Controller.update)))
        uses_old_call = any(isinstance(node, ast.Attribute) and node.attr == "qM" for node in ast.walk(tree))
    except (OSError, TypeError, SyntaxError):  # source unavailable (for example a zipped install): assume unfixed
        uses_old_call = True
    if not uses_old_call:
        return False  # robosuite was fixed upstream
    model = mujoco.MjModel.from_xml_string(_PROBE_XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    dst = np.zeros((model.nv, model.nv))
    try:
        mujoco.mj_fullM(model, dst, data.qM)  # exactly the call in robosuite's Controller.update
    except (AttributeError, TypeError):
        return True
    return False


def _patch_joint_address(mujoco, binding_utils) -> None:
    free, ball = int(mujoco.mjtJoint.mjJNT_FREE), int(mujoco.mjtJoint.mjJNT_BALL)
    one_dof = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))

    def make(address_field: str, free_dims: int, ball_dims: int):
        def get_address(self, name):
            joint_id = self.joint_name2id(name)
            joint_type = int(self.jnt_type[joint_id])  # int() is the fix: np.int32 vs enum membership fails >= 3.12
            address = getattr(self, address_field)[joint_id]
            if joint_type == free:
                ndim = free_dims
            elif joint_type == ball:
                ndim = ball_dims
            else:
                assert joint_type in one_dof
                ndim = 1
            return address if ndim == 1 else (address, address + ndim)

        return get_address

    binding_utils.MjModel.get_joint_qpos_addr = make("jnt_qposadr", 7, 4)
    binding_utils.MjModel.get_joint_qvel_addr = make("jnt_dofadr", 6, 3)


def _patch_mass_matrix(mujoco, binding_utils, controller_module) -> None:
    binding_utils.MjData.qM = property(lambda self: _LegacyMassMatrix(self._data))
    controller_module.mujoco = _MujocoProxy(mujoco)


def apply_compat() -> CompatReport:
    """Patch robosuite where the installed MuJoCo breaks it. Safe to call repeatedly and from several threads."""
    global _REPORT
    with _LOCK:
        if _REPORT is not None:
            return _REPORT
        import mujoco
        import robosuite
        from robosuite.controllers.parts import controller as controller_module
        from robosuite.utils import binding_utils

        applied: list[str] = []
        not_needed: list[str] = []

        if _joint_address_broken(mujoco, binding_utils):
            _patch_joint_address(mujoco, binding_utils)
            applied.append("joint-address-enum")
        else:
            not_needed.append("joint-address-enum")

        if _mass_matrix_broken(mujoco, controller_module):
            _patch_mass_matrix(mujoco, binding_utils, controller_module)
            applied.append("mass-matrix")
        else:
            not_needed.append("mass-matrix")

        _REPORT = CompatReport(mujoco.__version__, robosuite.__version__, tuple(applied), tuple(not_needed))
        return _REPORT
