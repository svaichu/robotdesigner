"""Engine side of ``menagerie.describe``: open a model and read the numbers an ``ExternalRobot`` needs."""

from __future__ import annotations

import warnings

import mujoco
import numpy as np

from ....config import EndEffectorRef, ExternalJoint, ExternalRobot, Pose
from ....errors import ConfigError
from ....simulators.mujoco.external import actuator_kind

O = mujoco.mjtObj
# Sites that conventional names mark as the end effector, in order of preference.
EE_SITES = ("attachment_site", "tcp_site", "pinch_site", "gripperframe", "pinch", "ee_gripper_link", "link_tcp")
_HINGE, _SLIDE, _FREE = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE), int(mujoco.mjtJoint.mjJNT_FREE))
_JOINT_TRN = int(mujoco.mjtTrn.mjTRN_JOINT)


def _end_effector_from_card(card: dict) -> EndEffectorRef:
    offset = card.get("offset")
    return EndEffectorRef(
        site=card.get("site"),
        body=card.get("body"),
        offset=Pose.identity() if offset is None else Pose(tuple(offset), (1.0, 0.0, 0.0, 0.0)),
    )


def describe(provider, name, entry, controlled, end_effector, keyframe, effort_limit) -> ExternalRobot:
    from . import DEFAULT, EXTRA, _entry_meta, _robot_meta, cards

    meta = _robot_meta(name)
    card = cards().get(name, {})
    entry = entry or card.get("entry") or meta.default_model
    if _entry_meta(meta, entry).kind == "scene":
        raise ConfigError("entry", f"{entry!r} is a scene: the compile and the scene presets bring floor and lights; use a robot entry")
    try:
        model = meta.spec(entry).compile()
    except Exception as e:
        raise ConfigError("name", f"menagerie/{name} could not be opened ({type(e).__name__}: {e}); {EXTRA}") from e
    jname = lambda j: mujoco.mj_id2name(model, O.mjOBJ_JOINT, j)

    # --- the driven joints and the actuator of each ---
    joint_act: dict[int, int] = {}
    for a in range(model.nu):
        if int(model.actuator_trntype[a]) == _JOINT_TRN:
            joint_act.setdefault(int(model.actuator_trnid[a, 0]), a)
    if controlled is None:
        controlled = card.get("controlled")
    if controlled is None:
        driven = list(joint_act)
    else:
        driven = []
        for n in controlled:
            j = mujoco.mj_name2id(model, O.mjOBJ_JOINT, n)
            if j < 0:
                raise ConfigError("controlled", f"menagerie/{name} has no joint {n!r}")
            if int(model.jnt_type[j]) == _FREE:
                raise ConfigError("controlled", f"{n!r} is a free joint: a free root is never driven")
            if j not in joint_act:
                raise ConfigError("controlled", f"joint {n!r} of menagerie/{name} has no actuator with a joint transmission")
            driven.append(j)
    driven = [j for j in driven if int(model.jnt_type[j]) in (_HINGE, _SLIDE)]
    if not driven:
        raise ConfigError("name", f"menagerie/{name} has no joint with an actuator that could be driven (actuators: {model.nu})")

    # --- ranges, effort limits ---
    joints = []
    for j in driven:
        a = joint_act[j]
        n = jname(j)
        rng = tuple(float(x) for x in model.jnt_range[j]) if model.jnt_limited[j] else None
        if model.actuator_forcelimited[a]:
            effort = float(np.abs(model.actuator_forcerange[a]).max())
        elif model.jnt_actfrclimited[j]:
            effort = float(np.abs(model.jnt_actfrcrange[j]).max())
        elif actuator_kind(model, a) == "torque" and model.actuator_ctrllimited[a]:
            # a plain motor: force = gain * ctrl * gear, so its ctrlrange is its force limit
            effort = float(np.abs(model.actuator_ctrlrange[a]).max() * model.actuator_gainprm[a, 0] * abs(model.actuator_gear[a, 0]))
        elif isinstance(effort_limit, dict) and n in effort_limit:
            effort = float(effort_limit[n])
        elif effort_limit is not None and not isinstance(effort_limit, dict):
            effort = float(effort_limit)
        else:
            raise ConfigError(
                "effort_limit",
                f"joint {n!r} of menagerie/{name} has no force limit in the model (actuator or joint): pass "
                f"effort_limit={{{n!r}: <N·m or N>}} or one number for all joints",
            )
        joints.append(ExternalJoint(name=n, range=rng, effort_limit=effort))

    # --- keyframe, home posture, root ---
    if keyframe is DEFAULT:
        keyframe = card["keyframe"] if "keyframe" in card else ("home" if mujoco.mj_name2id(model, O.mjOBJ_KEY, "home") >= 0 else None)
    key_id = None
    if keyframe is not None:
        key_id = mujoco.mj_name2id(model, O.mjOBJ_KEY, keyframe)
        if key_id < 0:
            raise ConfigError("keyframe", f"menagerie/{name} has no keyframe {keyframe!r}")
    state = model.qpos0 if key_id is None else model.key_qpos[key_id]
    home = []
    for j, spec in zip(driven, joints):
        q = float(state[model.jnt_qposadr[j]])
        if spec.range is not None and not spec.range[0] <= q <= spec.range[1]:
            clipped = min(max(q, spec.range[0]), spec.range[1])
            warnings.warn(f"menagerie/{name}: home value {q:.4f} of {spec.name!r} is outside its range {list(spec.range)}; clipped to {clipped:.4f}", stacklevel=3)
            q = clipped
        home.append(q)
    free = [j for j in range(model.njnt) if int(model.jnt_type[j]) == _FREE]
    home_root = None
    if free:
        a = model.jnt_qposadr[free[0]]
        home_root = Pose(tuple(float(x) for x in state[a : a + 3]), tuple(float(x) for x in state[a + 3 : a + 7]))

    # --- end effector ---
    if end_effector is None and "end_effector" in card:
        end_effector = _end_effector_from_card(card["end_effector"])
    if end_effector is None:
        for site in EE_SITES:
            if mujoco.mj_name2id(model, O.mjOBJ_SITE, site) >= 0:
                end_effector = EndEffectorRef(site=site, body=None, offset=Pose.identity())
                break

    kinds = {actuator_kind(model, joint_act[j]) for j in driven}
    return ExternalRobot(
        provider="menagerie", name=name, entry=entry, source=meta.oid, joints=tuple(joints), home_qpos=tuple(home),
        home_root=home_root, keyframe=keyframe, end_effector=end_effector,
        actuator_kind=next(iter(kinds)) if len(kinds) == 1 else "mixed",
    )
