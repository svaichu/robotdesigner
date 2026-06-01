from typing import Any, Callable, Dict, Optional, Type, Union

import mujoco

_mjx_envs: Dict[str, Type[Any]] = {}
_mjx_cfgs: Dict[str, Callable[[], Any]] = {}


def _ensure_default_mjx_env_registered() -> None:
	"""Register the built-in MJX env lazily to avoid eager dependency imports."""
	if "AbstractArmMjx" in _mjx_envs:
		return
	from robotdesigner.envs.abstract_mujoco_env_mjx import AbstractMjxEnv, default_config
	_mjx_envs["AbstractArmMjx"] = AbstractMjxEnv
	_mjx_cfgs["AbstractArmMjx"] = default_config


def register_mjx_environment(
	env_name: str,
	env_class: Type[Any],
	cfg_class: Callable[[], Any],
) -> None:
	"""Register an MJX environment and its default config."""
	_mjx_envs[env_name] = env_class
	_mjx_cfgs[env_name] = cfg_class


def get_mjx_default_config(env_name: str) -> Any:
	"""Return the default config for a registered MJX environment."""
	_ensure_default_mjx_env_registered()
	if env_name not in _mjx_cfgs:
		raise ValueError(
			f"Env '{env_name}' not found in default configs. Available configs: {list(_mjx_cfgs.keys())}"
		)
	return _mjx_cfgs[env_name]()


def load_mjx_environment(
	env_name: str,
	spec: mujoco.MjSpec,
	config: Optional[Any] = None,
	config_overrides: Optional[Dict[str, Union[str, int, list[Any]]]] = None,
) -> Any:
	"""Load a registered MJX environment by name."""
	_ensure_default_mjx_env_registered()
	if env_name not in _mjx_envs:
		raise ValueError(
			f"Env '{env_name}' not found. Available envs: {list(_mjx_envs.keys())}"
		)
	config = config or get_mjx_default_config(env_name)
	return _mjx_envs[env_name](spec=spec, config=config, config_overrides=config_overrides)


def list_mjx_environments() -> tuple[str, ...]:
	"""List registered MJX environment names."""
	_ensure_default_mjx_env_registered()
	return tuple(_mjx_envs.keys())


__all__ = [
	"register_mjx_environment",
	"get_mjx_default_config",
	"load_mjx_environment",
	"list_mjx_environments",
]
