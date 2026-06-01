import logging
import tempfile
from typing import Any, Dict, Tuple, Union

import mujoco
import numpy as np
from numpy.typing import NDArray

from robotdesigner.utils.myColumns import myColumns as Columns

try:
	from ray.rllib.env.env_context import EnvContext as _RayEnvContext
except ImportError:  # ray not installed
	_RayEnvContext = None


class EnvContext(dict):
	"""Minimal stand-in for ray.rllib.env.env_context.EnvContext.

	Behaves like a plain dict so existing code that iterates over it or calls
	dict(env_context) continues to work.  When ray IS installed the real
	EnvContext is used instead (see _normalize_env_context).
	"""

	def __init__(
		self,
		env_config: Dict[str, Any],
		worker_index: int = 0,
		vector_index: int = 0,
		remote: bool = False,
		num_workers: int = 0,
		recreated_worker: bool = False,
	):
		super().__init__(env_config)
		self.worker_index = worker_index
		self.vector_index = vector_index
		self.remote = remote
		self.num_workers = num_workers
		self.recreated_worker = recreated_worker

try:
	import gymnasium as gym
	from gymnasium import spaces
	from gymnasium.envs.registration import register as gym_register
	from gymnasium.envs.mujoco import MujocoEnv
	from gymnasium import utils
except Exception as e:  # pragma: no cover - import-time guard
	raise RuntimeError(
		"Gymnasium is required. Please install with `pip install gymnasium`."
	) from e



DEFAULT_CAMERA_CONFIG = {"trackbodyid": 0}

logger = logging.getLogger(__name__)

def _normalize_env_context(
	env_input: Union[EnvContext, Dict[str, Any], None]
) -> EnvContext:
	"""Convert any env_config-like input into an EnvContext.

	RLlib will pass an EnvContext when the env is provided as a class. However,
	if the env is gym-registered (string id), gymnasium will pass plain kwargs.
	This helper standardizes both cases so the env always receives an EnvContext
	with the config dict stored inside.
	"""
	_env_context_types = (EnvContext,) + ((_RayEnvContext,) if _RayEnvContext is not None else ())
	if isinstance(env_input, _env_context_types):
		return env_input

	env_kwargs: Dict[str, Any] = dict(env_input or {})
	worker_index = env_kwargs.pop("worker_index", 0)
	vector_index = env_kwargs.pop("vector_index", 0)
	remote = env_kwargs.pop("remote", False)
	num_workers = env_kwargs.pop("num_workers", None)
	recreated_worker = env_kwargs.pop("recreated_worker", False)

	return EnvContext(
		env_kwargs,
		worker_index=worker_index,
		vector_index=vector_index,
		remote=remote,
		num_workers=num_workers,
		recreated_worker=recreated_worker,
	)

class AbstractMujocoEnv(MujocoEnv, utils.EzPickle):
	"""
	Minimal Gymnasium environment for an ndof-link arm.

	Observation: concatenation of end-effector position (ee_pos), target position (target_pos), and proprioception (joint position, velocity, torque).
		shape: (3 + 3 + ndof*3,)
	Note: target position is set via set_target() and included in the observation.
	Action: normalized joint commands in [-1, 1] (one per DoF).
		shape: (ndof,)
	"""
	metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ],
		"render_fps": 50,
    }

	def __init__(
		self,
		env_config: Union[EnvContext, Dict[str, Any], None] = None,
		**overrides: Any,
	):
		"""Create a Mujoco env that reads its settings from env_config.

		The RLlib best-practice is to pass environment-specific values via
		`config.environment(env_config={...})`. We still support direct kwargs
		(e.g., `xml_filename`) so subclasses like `OneArmEnv` can set defaults.
		"""
		env_context = _normalize_env_context(env_config)
		config = dict(env_context)
		config.update(overrides)

		# Cache commonly overridden values before Mujoco initialization.
		self._frame_skip_override = config.get("frame_skip")
		self._camera_config_override = config.get("default_camera_config")
		self.render_mode = config.get("render_mode", "rgb_array")
		self._step_count = 0

		self.single_observation_space = None
		self.single_action_space = None

		self.load_config(config)

	def load_config(self, config: Dict[str, Any]):
		if config is None:
			raise ValueError("env_config is required to initialize the environment.")

		self._env_config = dict(config)

		spec: mujoco.MjSpec = config.get("spec")
		if spec is None:
			raise ValueError("env_config must contain a 'spec' key with a mujoco.MjSpec instance.")

		# Compile spec directly to read all model info — avoids xml roundtrip issues.
		_model = spec.compile()
		self.ndof = int(_model.nv)
		self.frame_skip = int(config.get("frame_skip", self._frame_skip_override or 2))

		# Derive render_fps from the spec's timestep and frame_skip so gymnasium's
		# assertion (1/dt == render_fps) always holds regardless of model settings.
		self.metadata = dict(self.metadata)
		self.metadata["render_fps"] = int(round(1.0 / (_model.opt.timestep * self.frame_skip)))

		# MujocoEnv.__init__ still needs a file path. Write cleaned xml to a temp
		# file (strip empty <default/> tags that MuJoCo's xml parser rejects).
		_xml_clean = spec.to_xml().replace("<default/>\n", "")
		tmp = tempfile.NamedTemporaryFile(suffix=".xml", delete=False, mode="w")
		tmp.write(_xml_clean)
		tmp.close()
		self._tmp_xml_path = tmp.name

		self.action_space = spaces.Box(
			low=-1, high=1, shape=(self.ndof,), dtype=np.float32
		)
		self.observation_space = spaces.Box(
			low=-np.inf, high=np.inf, shape=(6 + self.ndof * 3,), dtype=np.float32
		)
		camera_config = config.get(
			"default_camera_config", self._camera_config_override or DEFAULT_CAMERA_CONFIG
		)
		self.render_mode = config.get("render_mode", self.render_mode)

		MujocoEnv.__init__(
			self,
			self._tmp_xml_path,
			self.frame_skip,
			observation_space=self.observation_space,
			render_mode=self.render_mode,
			default_camera_config=camera_config,
		)

		utils.EzPickle.__init__(
			self,
			self._tmp_xml_path,
			self.frame_skip,
			camera_config,
			**{k: v for k, v in config.items() if k != "spec"},
		)

		nj = int(self.model.njnt)
		dof = int(self.model.nv)
		
		self._last_reset_state = np.zeros(nj, dtype=np.float32)
		self._last_action = np.zeros(nj, dtype=np.float32)

	def getJointLimits(self):
		joint_range = np.asarray(self.model.jnt_range, dtype=np.float32)
		nj = int(self.model.njnt)
		valid_joint_range = joint_range.shape[0] == int(self.model.njnt) and joint_range.shape[1] == 2
		if not valid_joint_range or not np.any(joint_range):
			# If ranges are unset or missing, fall back to [-pi, pi]
			joint_low = -np.pi * np.ones(nj, dtype=np.float32)
			joint_high = np.pi * np.ones(nj, dtype=np.float32)
		else:
			joint_low = joint_range[:, 0]
			joint_high = joint_range[:, 1]
			# If metadata reports fewer joints than DoFs, broadcast the last limit.
			if joint_low.shape[0] < nj:
				extra = nj - joint_low.shape[0]
				joint_low = np.concatenate([joint_low, np.repeat(joint_low[-1], extra)])
				joint_high = np.concatenate([joint_high, np.repeat(joint_high[-1], extra)])

		return joint_low, joint_high

	def setupMujoco(self):
		import os
		# import getpass; print(getpass.getuser())
		# Add an ICD config so that glvnd can pick up the Nvidia EGL driver.
		# This is usually installed as part of an Nvidia driver package, but the Colab
		# kernel doesn't install its driver via APT, and as a result the ICD is missing.
		# (https://github.com/NVIDIA/libglvnd/blob/master/src/EGL/icd_enumeration.md)
		NVIDIA_ICD_CONFIG_PATH = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
		if os.path.exists(NVIDIA_ICD_CONFIG_PATH):
		    with open(NVIDIA_ICD_CONFIG_PATH, 'w') as f:
		        f.write("""{
		        "file_format_version" : "1.0.0",
		        "ICD" : {
		            "library_path" : "libEGL_nvidia.so.0"
		        }
		    }
		    """)
		else:
			raise PermissionError(f"Is there an GPU available? ")

		# set env var 
		os.environ['MUJOCO_GL'] = 'egl'

		try:
			import mujoco
			mujoco.MjModel.from_xml_string('<mujoco/>')
			print('Mujoco is good to go!')
		except Exception as e:
		    raise e from RuntimeError(
		        'Something went wrong during installation. Check the shell output above '
		        'for more information.\n'
		        'If using a hosted Colab runtime, make sure you enable GPU acceleration '
		        'by going to the Runtime menu and selecting "Choose runtime type".')
	
	# --- Gymnasium MujocoEnv API ---
	def reset_model(self) -> Tuple[np.ndarray, Dict[str, Any]]:
		self._step_count = 0

		# Small random initialization around zero
		qpos_noise = self.np_random.uniform(low=-0.05, high=0.05, size=self.model.nq)
		qvel_noise = self.np_random.uniform(low=-0.01, high=0.01, size=self.model.nv)
		self._last_reset_state[...] = qpos_noise[: self._last_reset_state.shape[0]]

		# mujoco.mj_forward(self.model, self.data)
		self.set_state(qpos_noise, qvel_noise)

		observation = self._get_obs()
		return observation

	def set_target(self, target: NDArray[np.float32]):
		"""Set the target position for the environment."""
		self.target_pos = target
	
	def step(
		self, action: NDArray[np.float32]
		) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:

		self.do_simulation(action, self.frame_skip)
		self._last_action = action
		observation = self._get_obs()
		reward = self._get_reward(observation)
		Terminated = False
		Truncated = False
		info = self._get_info()

		self._step_count += 1
		# if self._step_count >= 10:
		# 	Truncated = True
		# 	# Terminated = True
		# 	info.setdefault("TimeLimit.truncated", True)

		return observation, reward, Terminated, Truncated, info

	def _get_reward(self, obs) -> float:
		ee_pos = obs[0:3]
		target_pos = obs[3:6]
		distance = np.linalg.norm(ee_pos - target_pos)
		reward = -distance  # Negative distance as reward
		return float(reward)

	def _get_reset_info(self) -> Dict[str, float]:
		"""Function that generates the `info` that is returned during a `reset()`."""
		info = self._get_info()
		# Also add velocity 
		# info[Columns.T] = float(self._step_count)
		info[Columns.LAST_RESET_STATE] = self._last_reset_state.copy()
		return info
	
	# --- Helpers ---
	def _get_obs(self) -> np.ndarray:
		# ee_frame_id = self.model.site_name2id("ee_frame")
		joint_state = self.data.qpos.copy()
		joint_vel = self.data.qvel.copy()
		torque = self.data.qfrc_actuator.copy()

		ee_frame_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_frame")
		ee_pos = self.data.site_xpos[ee_frame_id]  # World position
		ee_pos = ee_pos.copy().astype(np.float32)

		target_pos = self.target_pos.copy().astype(np.float32) if hasattr(self, 'target_pos') else np.ones(3, dtype=np.float32)

		proprioception = np.concatenate([joint_state, joint_vel, torque]).astype(np.float32)

		obs = np.concatenate([ee_pos, target_pos, proprioception]).astype(np.float32)

		return obs

	def _get_info(self):

		ee_frame_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_frame")
		ee_pos = self.data.site_xpos[ee_frame_id]  # World position
		ee_pos = ee_pos.copy().astype(np.float32)

		joint_state = self.data.qpos.copy()
		joint_vel = self.data.qvel.copy()
		torque = self.data.qfrc_actuator.copy()
		proprioception = np.concatenate([joint_state, joint_vel, torque]).astype(np.float32)

		
		target_pos = self.target_pos.copy().astype(np.float32) if hasattr(self, 'target_pos') else np.zeros(3, dtype=np.float32)

		last_action = self._last_action.copy()
		return {
			Columns.T: float(self._step_count),
			Columns.STATE: proprioception,
			Columns.CURR: ee_pos,
			Columns.TARGET: target_pos,
			Columns.ACTIONS: last_action,
		}

def make_env(spec: mujoco.MjSpec, **kwargs) -> "AbstractMujocoEnv":
	"""Create an AbstractMujocoEnv from a mujoco.MjSpec.

	Args:
		spec: A MjSpec describing the robot.
		**kwargs: Additional env config options (e.g. frame_skip, render_mode).
	"""
	env_context = _normalize_env_context({"spec": spec, **kwargs})
	return AbstractMujocoEnv(env_context)
