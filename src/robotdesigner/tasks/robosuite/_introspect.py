"""What robotdesigner needs to know about robosuite's environment classes, read from the library itself."""

from __future__ import annotations

import inspect
from typing import Any

from .params import RESERVED_ENV_KWARGS


def env_class(name: str) -> type | None:
    from robosuite.environments.base import REGISTERED_ENVS

    return REGISTERED_ENVS.get(name)


def is_single_arm_manipulation(cls: type) -> bool:
    from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
    from robosuite.environments.manipulation.two_arm_env import TwoArmEnv

    return issubclass(cls, ManipulationEnv) and not issubclass(cls, TwoArmEnv)


def single_arm_env_names() -> list[str]:
    from robosuite.environments.base import REGISTERED_ENVS

    return sorted(n for n, c in REGISTERED_ENVS.items() if is_single_arm_manipulation(c))


def constructor_params(cls: type) -> dict[str, inspect.Parameter]:
    """The named arguments ``cls(...)`` accepts, following ``**kwargs`` forwarding up the class chain.

    A class contributes its own ``__init__`` arguments; its parent's are added only if it takes ``**kwargs``
    (robosuite's task variants do, for example ``NutAssemblyRound``; ``Lift`` lists everything it accepts).
    The most derived class wins where several declare the same argument, so its default is the effective one.
    """
    params: dict[str, inspect.Parameter] = {}
    for klass in cls.__mro__:
        init = klass.__dict__.get("__init__")
        if init is None or klass is object:
            continue
        forwards = False
        for name, p in inspect.signature(init).parameters.items():
            if p.kind is p.VAR_KEYWORD:
                forwards = True
            elif name != "self" and p.kind is not p.VAR_POSITIONAL:
                params.setdefault(name, p)
        if not forwards:
            break
    return params


def resolved_env_kwargs(cls: type, user: dict[str, Any]) -> dict[str, Any]:
    """Every task argument the environment will run with: its own defaults, overridden by ``user``.

    Arguments robotdesigner sets itself are left out. Defaults that are not plain values are shown as ``repr``.
    """
    out: dict[str, Any] = {}
    for name, p in constructor_params(cls).items():
        if name in RESERVED_ENV_KWARGS or p.default is p.empty:
            continue
        out[name] = p.default
    out.update(user)
    plain = (bool, int, float, str, type(None), list, tuple, dict)
    return {k: (v if isinstance(v, plain) else repr(v)) for k, v in sorted(out.items())}
