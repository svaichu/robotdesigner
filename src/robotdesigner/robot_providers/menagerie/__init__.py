"""The robot provider ``menagerie``: robots of MuJoCo Menagerie, through the ``mujoco-menagerie`` library.

Importing this module imports no engine and not ``mujoco_menagerie``; each method imports what it needs when it is
called. No model file is copied into this repository: the library resolves and caches the models (its cache directory
is ``MENAGERIE_CACHE_DIR``, or ``MENAGERIE_ROOT`` points at a checkout) and the licences differ per model, so
:meth:`MenagerieRobots.license` reports the registry's licence of each.

``describe`` reads a model and returns an :class:`~robotdesigner.robot.ExternalRobot`; ``cards.json`` holds, per robot,
what cannot be derived (driven joints where a finger is a joint actuator, an end effector where the model has no
suitable site, a keyframe that is not called ``home``).
"""

from __future__ import annotations

import json
from importlib import resources
from typing import Any

from ...errors import ConfigError, Problem

__all__ = ["PROVIDER", "MenagerieRobots", "cards"]

DEFAULT = object()  # "use the model's keyframe called home, if it has one"
EXTRA = "pip install 'robotdesigner[menagerie]'"


def cards() -> dict[str, dict[str, Any]]:
    """The cards of ``cards.json`` by robot name."""
    data = json.loads(resources.files(__package__).joinpath("cards.json").read_text())
    return {k: v for k, v in data.items() if not k.startswith("_")}


def _library():
    try:
        import mujoco_menagerie
    except ImportError as e:
        raise ConfigError("provider", f"the menagerie provider needs the mujoco-menagerie library: {EXTRA}") from e
    return mujoco_menagerie


def _robot_meta(name: str):
    """The library's record of robot ``name`` (no download); ``ConfigError`` at ``name`` with the near match."""
    lib = _library()
    try:
        return lib.get(name)
    except lib.UnknownRobotError as e:
        raise ConfigError("name", str(e)) from e


def _entry_meta(meta, entry: str):
    try:
        return meta.entry(entry)
    except Exception as e:  # UnknownEntryPointError
        raise ConfigError("entry", str(e)) from e


class MenagerieRobots:
    """``RobotProvider`` of MuJoCo Menagerie."""

    def names(self) -> tuple[str, ...]:
        try:
            return tuple(_library().names())
        except ConfigError:
            return ()

    def license(self, name: str) -> str:
        """The licence string of the registry for robot ``name``."""
        return _robot_meta(name).license

    def describe(
        self,
        name: str,
        entry: str | None = None,
        *,
        controlled: tuple[str, ...] | list[str] | None = None,
        end_effector: Any = None,
        keyframe: Any = DEFAULT,
        effort_limit: float | dict[str, float] | None = None,
    ):
        """Open the model of robot ``name`` and return its :class:`ExternalRobot`.

        ``entry`` defaults to the card's, else the library's default model; a scene entry is refused. ``controlled``
        names the driven joints (default: the card's, else every joint with an actuator, in actuator order; free joints
        are never driven). ``end_effector`` is an ``EndEffectorRef`` (default: the card's, else a site with a
        conventional name, else none). ``keyframe`` defaults to the card's, else ``home`` if the model has it, else none.
        ``effort_limit`` supplies the limit of joints whose model has none: a number, or ``{joint name: limit}``.
        """
        _library()  # the library first: its absence is a ConfigError that names the extra
        from ._describe import describe

        return describe(self, name, entry, controlled, end_effector, keyframe, effort_limit)

    def spec(self, robot):
        """The editable ``mujoco.MjSpec`` of ``robot``, after checking ``robot.source`` against the library."""
        try:
            meta = _robot_meta(robot.name)
            _entry_meta(meta, robot.entry)
        except ConfigError as e:
            raise e.prefixed("robot") from None
        if meta.oid != robot.source:
            raise ConfigError("robot.source", STALE)
        try:
            return meta.spec(robot.entry)
        except Exception as e:  # a download that failed, a model that does not open
            raise ConfigError("robot", f"the model of menagerie/{robot.name} could not be opened: {type(e).__name__}: {e}") from e

    def validate(self, robot, simulator: str) -> list[Problem]:
        problems: list[Problem] = []
        try:
            meta = _robot_meta(robot.name)
            if _entry_meta(meta, robot.entry).kind == "scene":
                problems.append(Problem("robot.entry", f"{robot.entry!r} is a scene: the compile brings floor and lights"))
            if meta.oid != robot.source:
                problems.append(Problem("robot.source", STALE))
        except ConfigError as e:
            e = e.prefixed("robot")
            problems.append(Problem(e.path, e.reason))
        return problems

    def kinematics(self, robot):
        from ...simulators.mujoco.external import MujocoKinematics

        return MujocoKinematics(robot, self.spec(robot))


STALE = "the model changed since this config was written"
PROVIDER = MenagerieRobots()
