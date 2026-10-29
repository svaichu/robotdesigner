"""Robot embodiments as validated, hashable data, instantiated in batched RL environments.

``env = robotdesigner.make_env(env_config)`` is the single way to get an environment from an ``EnvConfig``.
"""

__version__ = "0.3.0"

__all__ = ["__version__", "make_env"]


def __getattr__(name):
    # Lazy, so that ``import robotdesigner`` (and ``robotdesigner.config``) import no engine.
    if name == "make_env":
        from .envs import make_env

        return make_env
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(set(globals()) | {"make_env"})
