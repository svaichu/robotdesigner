"""``BaseEnvConfig``: a complete, runnable environment description refined with accumulating section calls.

The builder layer, like RLlib's ``AlgorithmConfig`` in ergonomics only: no ``ray``, no trainer. ``BaseEnvConfig()`` is a
one-link arm in the ``empty`` scene doing the ``empty/idle`` task (the table in ``defaults.py``); ``.robot()``,
``.control()``, ``.task()`` and ``.sim()`` each return a new config that changes what the call writes and keeps the rest;
``resolve()`` returns the frozen, validated :class:`EnvConfig` with its ``env_hash``. Defaults live here and nowhere
else: ``EnvConfig`` and the configs under it keep requiring every field. numpy only; the engine is imported by ``build()``.
"""

from __future__ import annotations

from dataclasses import fields
from typing import Any

from .control import ControlConfig, FixedGains
from .defaults import DEFAULTS, DEFAULTS_VERSION
from .env import EnvConfig
from .errors import ConfigError, Problem
from .providers import get_task_provider
from .robot import ParametricArm, Pose
from .sim import SIMULATOR_OPTIONS, SimConfig
from .specs import NOT_SET, Chain
from .task import TaskConfig

__all__ = ["BaseEnvConfig"]

_CONTROL_FIELDS = ("mode", "gains", "action_scale")
_TASK_FIELDS = ("horizon", "success_tolerance", "target_region", "mount")
_SIM_FIELDS = tuple(f.name for f in fields(SimConfig))
_GOAL_FIELDS = ("success_tolerance", "target_region")


def _unknown(section: str, key: str, allowed) -> ConfigError:
    return ConfigError(f"{section}.{key}", f"unknown field; expected one of {sorted(allowed)}")


def _check_keys(section: str, given: dict, allowed) -> None:
    for key in given:
        if key not in allowed:
            raise _unknown(section, key, allowed)


def _norm(v: Any) -> Any:
    """Lists and tuples compare equal, for ``diff``."""
    return tuple(_norm(x) for x in v) if isinstance(v, (list, tuple)) else v


def _default_options(simulator: str) -> Any:
    cls = SIMULATOR_OPTIONS.get(simulator)
    if cls is None:
        raise ConfigError("sim.simulator", f"no options registered for simulator {simulator!r}; known: {sorted(SIMULATOR_OPTIONS)}")
    default = getattr(cls, "default", None)
    return NOT_SET if default is None else default()


class BaseEnvConfig:
    """See the module docstring. Immutable by convention: every call returns a new object, the receiver is unchanged."""

    __slots__ = ("_robot", "_control", "_task", "_sim", "_written")

    def __init__(self):
        d = DEFAULTS
        provider, name = d["task"]["provider"], d["task"]["name"]
        params_cls = get_task_provider(provider).params_class(name)
        self._robot: Chain | ParametricArm = Chain(**d["robot"])
        self._control = {"mode": d["control"]["mode"], "gains": FixedGains(d["control"]["kp"], d["control"]["kd"]),
                         "action_scale": d["control"]["action_scale"]}
        self._task = {"params": params_cls(name=name), "horizon": d["task"]["horizon"], "success_tolerance": None,
                      "target_region": None, "mount": NOT_SET}
        self._sim = {k: d["sim"][k] for k in _SIM_FIELDS if k != "options"}
        self._sim["options"] = _default_options(d["sim"]["simulator"])
        self._written: frozenset[str] = frozenset()

    # --- copying ---------------------------------------------------------------------

    def _copy(self, *, robot=None, control=None, task=None, sim=None, written=None) -> "BaseEnvConfig":
        new = object.__new__(BaseEnvConfig)
        new._robot = self._robot if robot is None else robot
        new._control = self._control if control is None else control
        new._task = self._task if task is None else task
        new._sim = self._sim if sim is None else sim
        new._written = self._written if written is None else written
        return new

    # --- section calls ---------------------------------------------------------------

    def robot(self, robot: Any = NOT_SET, **fields_: Any) -> "BaseEnvConfig":
        """Refine the robot. ``robot`` is a :class:`Chain` (written fields change, the rest carries), a ready
        ``ParametricArm`` (a fixed robot: it replaces the section) or omitted; keywords are fields of ``Chain``."""
        allowed = [f.name for f in fields(Chain)]
        _check_keys("robot", fields_, allowed)
        written = set(self._written)
        if isinstance(robot, ParametricArm):
            if fields_:
                raise ConfigError("robot", "a ready ParametricArm cannot be combined with Chain fields")
            written = {w for w in written if not w.startswith("robot")} | {"robot"}
            return self._copy(robot=robot, written=frozenset(written))
        if robot is not NOT_SET and not isinstance(robot, Chain):
            raise ConfigError("robot", f"expected a Chain or a ParametricArm, got {type(robot).__name__}")
        if isinstance(self._robot, Chain):
            base = self._robot
        else:  # a fixed robot is replaced by the default chain with the written fields on top
            base = Chain(**DEFAULTS["robot"])
            written = {w for w in written if not w.startswith("robot")}
        merged, new = base, set()
        for spec in (robot, Chain(**fields_)):
            if spec is not NOT_SET:
                merged = merged.merge(spec)
                new |= {f"robot.{n}" for n in spec.written()}
        written = {w for w in written if w[6:] in merged.written() or not w.startswith("robot.")} | new
        return self._copy(robot=merged, written=frozenset(written))

    def control(self, control: Any = NOT_SET, **fields_: Any) -> "BaseEnvConfig":
        """Refine the control: keywords ``mode``, ``gains``, ``action_scale``, or a whole ``ControlConfig``."""
        _check_keys("control", fields_, _CONTROL_FIELDS)
        values = dict(self._control)
        changed = dict(fields_)
        if control is not NOT_SET:
            if not isinstance(control, ControlConfig):
                raise ConfigError("control", f"expected a ControlConfig, got {type(control).__name__}")
            changed = {**{k: getattr(control, k) for k in _CONTROL_FIELDS}, **changed}
        values.update(changed)
        return self._copy(control=values, written=self._written | {f"control.{k}" for k in changed})

    def task(self, params: Any = NOT_SET, **fields_: Any) -> "BaseEnvConfig":
        """Refine the task: a provider's params object (it is the task), keywords ``horizon``, ``success_tolerance``,
        ``target_region``, ``mount``, or a whole ``TaskConfig``. A new params class resets ``success_tolerance`` and
        ``target_region`` (they belong to goal tasks) unless the call writes them. A written ``mount`` is pinned."""
        _check_keys("task", fields_, _TASK_FIELDS)
        values = dict(self._task)
        changed = dict(fields_)
        if isinstance(params, TaskConfig):
            changed = {**{k: getattr(params, k) for k in _TASK_FIELDS}, **changed}
            params = params.params
        if params is not NOT_SET:
            if type(params) is not type(values["params"]):
                for k in _GOAL_FIELDS:
                    changed.setdefault(k, None)
            changed["params"] = params
        values.update(changed)
        return self._copy(task=values, written=self._written | {f"task.{k}" for k in changed})

    def sim(self, sim: Any = NOT_SET, **fields_: Any) -> "BaseEnvConfig":
        """Refine the simulation: keywords are the fields of ``SimConfig``, or give a whole ``SimConfig``. Changing
        ``simulator`` resets ``options`` to that simulator's ``default()`` unless ``options=`` is given."""
        _check_keys("sim", fields_, _SIM_FIELDS)
        values = dict(self._sim)
        changed = dict(fields_)
        if sim is not NOT_SET:
            if not isinstance(sim, SimConfig):
                raise ConfigError("sim", f"expected a SimConfig, got {type(sim).__name__}")
            changed = {**{k: getattr(sim, k) for k in _SIM_FIELDS}, **changed}
        if "simulator" in changed and changed["simulator"] != values["simulator"] and "options" not in changed:
            changed["options"] = _default_options(changed["simulator"])
        values.update(changed)
        return self._copy(sim=values, written=self._written | {f"sim.{k}" for k in changed})

    # --- resolution ------------------------------------------------------------------

    def _default_mount(self) -> Pose:
        params = self._task["params"]
        try:
            return get_task_provider(params.provider).default_mount(params.name)
        except (ConfigError, AttributeError):
            return Pose.identity()  # TaskConfig reports what is wrong with the params itself

    def _build(self) -> tuple[list[Problem], EnvConfig | None]:
        problems: list[Problem] = []
        parts: dict[str, Any] = {}

        def attempt(section: str, make) -> None:
            try:
                parts[section] = make()
            except ConfigError as e:
                p = e.prefixed(section)
                problems.append(Problem(p.path, p.reason))
            except (TypeError, ValueError, AttributeError) as e:
                problems.append(Problem(section, f"{type(e).__name__}: {e}"))

        attempt("robot", lambda: self._robot if isinstance(self._robot, ParametricArm) else self._robot.arm())
        attempt("control", lambda: ControlConfig(**self._control))

        def task():
            t = dict(self._task)
            if t["mount"] is NOT_SET:
                t["mount"] = self._default_mount()
            return TaskConfig(**t)

        def sim():
            s = dict(self._sim)
            if s["options"] is NOT_SET:
                raise ConfigError("options", f"simulator {s['simulator']!r} has no default options; pass options=")
            return SimConfig(**s)

        attempt("task", task)
        attempt("sim", sim)
        if problems:
            return problems, None
        try:
            return [], EnvConfig(robot=parts["robot"], control=parts["control"], task=parts["task"], sim=parts["sim"])
        except ConfigError as e:
            return [Problem(e.path, e.reason)], None

    def validate(self) -> list[Problem]:
        """Every problem of the config as ``Problem(path, message)``, without raising. Checks the config only; what a
        simulator or a task provider cannot run is reported by ``make_env``."""
        return self._build()[0]

    def resolve(self) -> EnvConfig:
        """The frozen :class:`EnvConfig`, every value explicit (the task's default mount filled in). Raises one
        ``ConfigError`` that lists every problem with its dotted path (``.problems`` holds them)."""
        problems, cfg = self._build()
        if cfg is not None:
            return cfg
        if len(problems) == 1:
            err = ConfigError(problems[0].path, problems[0].message)
        else:
            lines = "\n".join(f"  - {p}" for p in problems)
            err = ConfigError("", f"{len(problems)} problems in the config:\n{lines}")
        err.problems = tuple(problems)
        raise err

    def build(self) -> Any:
        """``make_env(self.resolve())``: the one construction path. Imports the engine here."""
        from ..envs import make_env

        return make_env(self.resolve())

    # --- inspection ------------------------------------------------------------------

    def _fields(self, *, resolved_mount: bool) -> list[tuple[str, Any, str]]:
        w = self._written
        src = lambda path: "set" if path in w else "default"  # noqa: E731
        out: list[tuple[str, Any, str]] = []
        if isinstance(self._robot, ParametricArm):
            out.append(("robot", self._robot, "set"))
        else:
            out += [(f"robot.{n}", getattr(self._robot, n), src(f"robot.{n}")) for n in self._robot.written()]
        out += [(f"control.{k}", self._control[k], src(f"control.{k}")) for k in _CONTROL_FIELDS]
        out.append(("task.params", self._task["params"], src("task.params")))
        for k in _TASK_FIELDS:
            v, s = self._task[k], src(f"task.{k}")
            if k == "mount" and v is NOT_SET:
                v, s = (self._default_mount() if resolved_mount else NOT_SET), "default (task)"
            out.append((f"task.{k}", v, s))
        out += [(f"sim.{k}", self._sim[k], src(f"sim.{k}")) for k in _SIM_FIELDS]
        return out

    def explain(self) -> str:
        """Every field with its value and where it comes from: ``set``, ``default`` or ``default (task)``."""
        rows = self._fields(resolved_mount=True)
        width = max(len(p) for p, _, _ in rows)
        head = f"BaseEnvConfig (defaults version {DEFAULTS_VERSION})"
        return "\n".join([head] + [f"  {p:<{width}}  {v!r}  [{s}]" for p, v, s in rows])

    def diff(self) -> dict[str, tuple[Any, Any]]:
        """``{path: (default, value)}`` against the pristine ``BaseEnvConfig()`` of this defaults version. A field
        that only one side has is ``NOT_SET`` on the other; an unpinned mount is not compared."""
        mine = {p: v for p, v, _ in self._fields(resolved_mount=False)}
        base = {p: v for p, v, _ in BaseEnvConfig()._fields(resolved_mount=False)}
        out = {}
        for path in [*base, *(p for p in mine if p not in base)]:
            a, b = base.get(path, NOT_SET), mine.get(path, NOT_SET)
            if _norm(a) != _norm(b):
                out[path] = (a, b)
        return out

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, BaseEnvConfig):
            return NotImplemented
        return (self._robot, self._control, self._task, self._sim) == (other._robot, other._control, other._task, other._sim)

    __hash__ = None  # type: ignore[assignment]

    def __repr__(self) -> str:
        changed = ", ".join(sorted(self._written)) or "nothing"
        return f"BaseEnvConfig(defaults v{DEFAULTS_VERSION}; set: {changed})"
