"""``BaseEnvConfig``: a complete, runnable environment description refined with accumulating section calls.

The builder layer, like RLlib's ``AlgorithmConfig`` in ergonomics only: no ``ray``, no trainer. ``BaseEnvConfig()`` is a
one-link arm in the ``empty`` scene doing the ``empty/idle`` task (the table in ``defaults.py``); ``.robot()``,
``.control()``, ``.task()`` and ``.sim()`` each return a new config that changes what the call writes and keeps the rest;
``resolve()`` returns the frozen, validated :class:`EnvConfig` with its ``env_hash``. Defaults live here and nowhere
else: ``EnvConfig`` and the configs under it keep requiring every field. numpy only; the engine is imported by ``build()``.
"""

from __future__ import annotations

import json
import warnings
from dataclasses import fields
from pathlib import Path
from typing import Any, Mapping, Sequence

from ._canon import check_keys, within
from .control import ControlConfig, FixedGains
from .defaults import DEFAULTS, DEFAULTS_VERSION
from .env import EnvConfig
from .errors import ConfigError, Problem
from .providers import get_task_provider
from .robot import ParametricArm, Pose, ToolSpec
from .sim import SIMULATOR_OPTIONS, SimConfig
from .specs import NOT_SET, Chain
from .task import TaskConfig, migrate_task_dict

__all__ = ["BaseEnvConfig"]

_CONTROL_FIELDS = ("mode", "gains", "action_scale")
_TASK_FIELDS = ("max_episode_steps", "success_tolerance", "target_region", "mount")
_SIM_FIELDS = tuple(f.name for f in fields(SimConfig))
_GOAL_FIELDS = ("success_tolerance", "target_region")


def _unknown(section: str, key: str, allowed) -> ConfigError:
    hint = " ('horizon' was renamed to 'max_episode_steps')" if (section, key) == ("task", "horizon") else ""
    return ConfigError(f"{section}.{key}", f"unknown field{hint}; expected one of {sorted(allowed)}")


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


_SECTIONS = ("robot", "control", "task", "sim")
_CHAIN_FIELDS = tuple(f.name for f in fields(Chain))
_PAIRS = (("link_length", "total_length"), ("mass", "total_mass"))


def _jsonable(v: Any) -> Any:
    """A plain JSON-able value: objects with ``to_dict`` are converted, tuples become lists."""
    if hasattr(v, "to_dict"):
        return v.to_dict()
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, Mapping):
        return {k: _jsonable(x) for k, x in v.items()}
    return v


def _tuplify(v: Any) -> Any:
    return tuple(_tuplify(x) for x in v) if isinstance(v, (list, tuple)) else v


def _parse_override(item: str) -> tuple[str, Any]:
    """``"path=value"``: the value is a JSON literal, or a string if it is not one."""
    path, sep, raw = item.partition("=")
    if not sep or not path.strip():
        raise ConfigError("", f"override {item!r} is not of the form path=value")
    try:
        return path.strip(), json.loads(raw)
    except ValueError:
        return path.strip(), raw


def _yaml():
    try:
        import yaml
    except ImportError as e:
        raise ImportError("YAML configs need the optional extra: pip install 'robotdesigner[yaml]'") from e
    return yaml


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
        self._task = {"params": params_cls(name=name), "max_episode_steps": d["task"]["max_episode_steps"], "success_tolerance": None,
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
        """Refine the task: a provider's params object (it is the task), keywords ``max_episode_steps``, ``success_tolerance``,
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

    # --- serialization -----------------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        """Every builder field as plain data, with ``defaults_version``. A mount that was not pinned is omitted: it is
        the task's default, applied by ``resolve()``. The record to keep with a run is ``resolve().to_dict()``."""
        robot = (self._robot.to_dict() if isinstance(self._robot, ParametricArm)
                 else {"kind": "chain", **{n: _jsonable(getattr(self._robot, n)) for n in self._robot.written()}})
        t = self._task
        task = {"provider": t["params"].provider, "params": t["params"].to_dict(), "max_episode_steps": t["max_episode_steps"],
                "success_tolerance": t["success_tolerance"], "target_region": _jsonable(t["target_region"])}
        if t["mount"] is not NOT_SET:
            task["mount"] = _jsonable(t["mount"])
        sim = {k: self._sim[k] for k in _SIM_FIELDS if k != "options"}
        if self._sim["options"] is not NOT_SET:
            sim["options"] = self._sim["options"].to_dict()
        control = {"mode": self._control["mode"], "gains": _jsonable(self._control["gains"]),
                   "action_scale": self._control["action_scale"]}
        return {"defaults_version": DEFAULTS_VERSION, "robot": robot, "control": control, "task": task, "sim": sim}

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "BaseEnvConfig":
        """Inverse of ``to_dict``. Sections and keys left out take the base's value, so a hand-written file can be short.
        Warns when ``defaults_version`` differs from this package's."""
        check_keys(d, set(), "", optional={"defaults_version", *_SECTIONS})
        version = d.get("defaults_version", DEFAULTS_VERSION)
        if version != DEFAULTS_VERSION:
            warnings.warn(
                f"config written with defaults version {version}, this is version {DEFAULTS_VERSION}: values it left "
                "out take the new defaults", stacklevel=2,
            )
        cfg = cls()
        pristine = cfg.to_dict()
        for section in _SECTIONS:
            if section in d:
                given = d[section]
                if not isinstance(given, Mapping):
                    raise ConfigError(section, f"expected a mapping, got {type(given).__name__}")
                if section == "task":
                    given = within("task", migrate_task_dict, given)
                base = dict(pristine[section])
                if section == "sim":
                    base.pop("options")  # the options of the simulator written, or its defaults
                if section == "robot":
                    base = {}  # a chain is merged onto the default chain, an arm stands alone
                cfg = cfg._with_section(section, {**base, **given}, merge_chain=True)
        return cfg._with_written(cfg._diff_paths())

    def _diff_paths(self) -> frozenset[str]:
        return frozenset(self.diff())

    def _with_written(self, paths) -> "BaseEnvConfig":
        keep = {w for w in paths if not w.startswith("robot.") or w[6:] in getattr(self._robot, "written", lambda: ())()}
        return self._copy(written=frozenset(keep))

    def _with_section(self, section: str, d: Mapping[str, Any], merge_chain: bool = False) -> "BaseEnvConfig":
        """The config with ``section`` replaced by what the dict ``d`` says (validated by the section's own classes)."""
        if section == "robot":
            if d.get("kind", "chain") == "parametric_arm":
                return self._copy(robot=within("robot", ParametricArm.from_dict, d))
            check_keys(d, set(), "robot", optional={"kind", *_CHAIN_FIELDS})
            if d.get("kind", "chain") != "chain":
                raise ConfigError("robot.kind", f"expected 'chain' or 'parametric_arm', got {d['kind']!r}")
            given = {}
            for k, v in d.items():
                if k == "kind":
                    continue
                if k == "end_effector" and isinstance(v, Mapping):
                    v = within(f"robot.{k}", Pose.from_dict, v)
                elif k == "tool" and isinstance(v, Mapping):
                    v = within(f"robot.{k}", ToolSpec.from_dict, v)
                given[k] = _tuplify(v) if isinstance(v, (list, tuple)) else v
            spec = Chain(**given)
            return self._copy(robot=Chain(**DEFAULTS["robot"]).merge(spec) if merge_chain else spec)
        if section == "control":
            c = within("control", ControlConfig.from_dict, d)
            return self._copy(control={k: getattr(c, k) for k in _CONTROL_FIELDS})
        if section == "task":
            check_keys(d, {"provider", "params", "max_episode_steps", "success_tolerance", "target_region"}, "task", {"mount"})
            probe = within("task", TaskConfig.from_dict, {**d, "mount": d.get("mount", Pose.identity().to_dict())})
            task = {"params": probe.params, "max_episode_steps": probe.max_episode_steps, "success_tolerance": probe.success_tolerance,
                    "target_region": probe.target_region,
                    "mount": probe.mount if "mount" in d else NOT_SET}
            return self._copy(task=task)
        d = dict(d)
        check_keys(d, set(), "sim", optional=set(_SIM_FIELDS))
        full = {k: v for k, v in d.items() if k != "options"}
        sim = {k: self._sim[k] for k in _SIM_FIELDS if k != "options"} | full
        options = d.get("options")
        if options is None:
            options = _default_options(sim["simulator"])
            if options is NOT_SET:
                raise ConfigError("sim.options", f"simulator {sim['simulator']!r} has no default options; give options")
            options = options.to_dict()
        probe = within("sim", SimConfig.from_dict, {**sim, "options": options})
        return self._copy(sim={k: getattr(probe, k) for k in _SIM_FIELDS})

    # --- overrides ---------------------------------------------------------------------

    def with_overrides(self, overrides: Mapping[str, Any] | Sequence[str]) -> "BaseEnvConfig":
        """A new config with dotted-path overrides applied, with the semantics of the section calls (they accumulate).

        ``overrides`` is a mapping ``{"sim.num_envs": 64, "sim.options.self_collision": True, "robot.total_length": 1.2}``
        or a list of CLI strings ``["sim.num_envs=64", "robot.axes=zyy"]`` (the value is a JSON literal, or a string if
        it is not one). Paths reach nested fields (``sim.options.*``, ``task.params.*``, ``control.gains.*``). Unknown
        keys and sections raise ``ConfigError`` with the dotted path; writing one of ``link_length`` / ``total_length``
        (or ``mass`` / ``total_mass``) clears the other; changing ``sim.simulator`` resets ``sim.options`` to that
        simulator's defaults unless ``sim.options.*`` is given too. Each touched section is rebuilt and validated by
        its own classes."""
        if isinstance(overrides, Mapping):
            items = list(overrides.items())
        elif isinstance(overrides, str) or not hasattr(overrides, "__iter__"):
            raise ConfigError("", "overrides must be a mapping or a list of 'path=value' strings")
        else:
            items = []
            for o in overrides:
                if not isinstance(o, str):
                    raise ConfigError("", f"override {o!r} is not a 'path=value' string")
                items.append(_parse_override(o))
        state = self.to_dict()
        touched: dict[str, set[str]] = {}
        # a simulator change first, so that options edits apply to the options of the new simulator
        items.sort(key=lambda kv: kv[0] != "sim.simulator")
        written_now: set[str] = set()
        for path, value in items:
            parts = str(path).split(".")
            if parts[0] not in _SECTIONS:
                raise ConfigError(parts[0], f"unknown section; expected one of {list(_SECTIONS)}")
            if len(parts) == 1:
                raise ConfigError(path, "give a field of the section (for example sim.num_envs)")
            section = state[parts[0]]
            if path == "sim.simulator" and value != section["simulator"]:
                new = _default_options(value)
                if new is NOT_SET:
                    if not any(p.startswith("sim.options") for p, _ in items):
                        raise ConfigError("sim.options", f"simulator {value!r} has no default options; give sim.options.*")
                    section.pop("options", None)
                else:
                    section["options"] = new.to_dict()
            self._set_path(state, parts, _jsonable(value), written_now)
            touched.setdefault(parts[0], set()).add(".".join(parts[:2]))
        cfg = self
        for section in _SECTIONS:
            if section in touched:
                cfg = cfg._with_section(section, state[section])
        written = set(self._written) | {w for ws in touched.values() for w in ws}
        if isinstance(cfg._robot, Chain):
            written = {w for w in written if not w.startswith("robot.") or w[6:] in cfg._robot.written()}
        return cfg._copy(written=frozenset(written))

    @staticmethod
    def _set_path(state: dict, parts: list[str], value: Any, written_now: set[str]) -> None:
        node, walked = state, []
        for i, key in enumerate(parts[:-1]):
            walked.append(key)
            nxt = node.get(key) if isinstance(node, dict) else None
            if not isinstance(nxt, dict):
                raise ConfigError(".".join(parts[: i + 1]), "has no fields to reach into" if key in node else
                                  f"unknown field; expected one of {sorted(node)}")
            node = nxt
        key, section = parts[-1], parts[0]
        path = ".".join(parts)
        chain_level = section == "robot" and len(parts) == 2 and node.get("kind", "chain") == "chain"
        allowed = set(node) | ({*_CHAIN_FIELDS} if chain_level else set()) | ({"mount"} if path == "task.mount" else set())
        if key not in allowed:
            raise ConfigError(path, f"unknown field; expected one of {sorted(allowed)}")
        node[key] = value
        if chain_level:
            written_now.add(key)
            for a, b in _PAIRS:
                for x, y in ((a, b), (b, a)):
                    if key == x and y not in written_now:
                        node.pop(y, None)

    # --- sweeps ------------------------------------------------------------------------

    def sweep(self, *, paired: Mapping[str, Any] | None = None, grid: Mapping[str, Any] | None = None):
        """A :class:`Family` of configs: ``paired`` axes are zipped (equal lengths), ``grid`` axes form a Cartesian
        product, and the paired group multiplies with the grid. Keys are dotted paths with the semantics of
        ``with_overrides``. Order: the paired group first, then the grid axes in the order given, the last one fastest.
        Items whose resolved ``env_hash`` repeats are dropped (see ``Family.notes``); an invalid item raises at
        expansion, naming its index and overrides."""
        from .sweep import expand

        return expand(self, paired, grid)

    # --- files -------------------------------------------------------------------------

    def dump(self, path: str | Path, format: str | None = None) -> None:
        """Write ``to_dict()`` as JSON, or as YAML for ``.yaml`` / ``.yml`` (needs the ``yaml`` extra)."""
        path = Path(path)
        fmt = format or ("yaml" if path.suffix.lower() in (".yaml", ".yml") else "json")
        if fmt == "yaml":
            path.write_text(_yaml().safe_dump(self.to_dict(), sort_keys=False))
        elif fmt == "json":
            path.write_text(json.dumps(self.to_dict(), indent=2) + "\n")
        else:
            raise ConfigError("format", f"expected 'json' or 'yaml', got {fmt!r}")

    @classmethod
    def load(cls, path: str | Path, format: str | None = None) -> "BaseEnvConfig":
        """Read what ``dump`` wrote."""
        path = Path(path)
        fmt = format or ("yaml" if path.suffix.lower() in (".yaml", ".yml") else "json")
        if fmt == "yaml":
            return cls.from_dict(_yaml().safe_load(path.read_text()))
        if fmt == "json":
            return cls.from_dict(json.loads(path.read_text()))
        raise ConfigError("format", f"expected 'json' or 'yaml', got {fmt!r}")

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
