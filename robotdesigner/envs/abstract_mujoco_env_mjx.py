import logging
import tempfile
from typing import Any, Dict, Optional, Union

import jax
import jax.numpy as jp
import mujoco
from mujoco import mjx
from ml_collections import config_dict
import numpy as np

from mujoco_playground._src import mjx_env
from mujoco_playground._src.mjx_env import State

from robotdesigner.utils.myColumns import myColumns as Columns

logger = logging.getLogger(__name__)


def default_config() -> config_dict.ConfigDict:
	"""Default environment config.

	ctrl_dt: control timestep (one env step).
	sim_dt:  physics simulation timestep (matches MuJoCo default).
	n_substeps = ctrl_dt / sim_dt  (analogous to the old frame_skip).
	episode_length: number of env steps before done is emitted.
	"""
	return config_dict.create(
		ctrl_dt=0.004,  # 2 substeps × 0.002 s MuJoCo default
		sim_dt=0.002,
		episode_length=50,
	)


class AbstractMjxEnv(mjx_env.MjxEnv):
	"""MJX-based environment for an ndof-link arm.

	Functional API (mujoco_playground style):
		state = env.reset(rng)       # rng: jax.Array (PRNGKey)
		state = env.step(state, act) # act: jax.Array shape (ndof,)

	Observation: [ee_pos (3), target_pos (3), qpos (ndof), qvel (ndof), torque (ndof)]
	Action:      normalized joint commands in [-1, 1], shape (ndof,)
	"""

	def __init__(
		self,
		spec: mujoco.MjSpec,
		config: Optional[config_dict.ConfigDict] = None,
		config_overrides: Optional[Dict[str, Union[str, int, list[Any]]]] = None,
	):
		if config is None:
			config = default_config()
		super().__init__(config, config_overrides)

		mj_model = spec.compile()
		mj_model.opt.timestep = self.sim_dt
		self.ndof = int(mj_model.nv)

		_xml_clean = spec.to_xml().replace("<default/>\n", "")
		tmp = tempfile.NamedTemporaryFile(suffix=".xml", delete=False, mode="w")
		tmp.write(_xml_clean)
		tmp.close()
		self._xml_path = tmp.name

		self._mj_model = mj_model
		self._mjx_model = mjx.put_model(mj_model)

		# Default target; update with set_target() before calling reset().
		self._target_pos = np.zeros(3, dtype=np.float32)

	# --- Abstract properties required by MjxEnv ---

	@property
	def xml_path(self) -> str:
		return self._xml_path

	@property
	def action_size(self) -> int:
		return self.ndof

	@property
	def mj_model(self) -> mujoco.MjModel:
		return self._mj_model

	@property
	def mjx_model(self) -> mjx.Model:
		return self._mjx_model

	# --- MjxEnv functional API ---

	def reset(self, rng: jax.Array) -> State:
		rng, rng_qpos, rng_qvel = jax.random.split(rng, 3)
		qpos = jax.random.uniform(rng_qpos, (self._mj_model.nq,), minval=-0.05, maxval=0.05)
		qvel = jax.random.uniform(rng_qvel, (self._mj_model.nv,), minval=-0.01, maxval=0.01)

		data = mjx_env.make_data(self._mj_model, qpos=qpos, qvel=qvel)
		data = mjx.forward(self._mjx_model, data)

		target_pos = jp.array(self._target_pos, dtype=jp.float32)
		obs = self._get_obs(data, target_pos)

		ee_frame_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SITE, "ee_frame")
		info = {
			Columns.T: jp.zeros(()),
			Columns.TARGET: target_pos,
			Columns.CURR: data.site_xpos[ee_frame_id].astype(jp.float32),
			Columns.STATE: jp.concatenate([qpos, qvel, jp.zeros_like(data.qfrc_actuator)]).astype(jp.float32),
			Columns.ACTIONS: jp.zeros(self.ndof, dtype=jp.float32),
			Columns.LAST_RESET_STATE: qpos[: self.ndof].astype(jp.float32),
		}
		return State(
			data=data,
			obs=obs,
			reward=jp.zeros(()),
			done=jp.zeros(()),
			metrics={},
			info=info,
		)

	def step(self, state: State, action: jax.Array) -> State:
		action = jp.clip(action, -1.0, 1.0)
		data = mjx_env.step(self._mjx_model, state.data, action, self.n_substeps)

		target_pos = state.info[Columns.TARGET]
		next_t = state.info[Columns.T] + 1.0
		ee_frame_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SITE, "ee_frame")
		obs = self._get_obs(data, target_pos)
		reward = self._get_reward(obs)
		done = (next_t >= self._config.episode_length).astype(reward.dtype)

		info = {
			**state.info,
			Columns.T: next_t,
			Columns.CURR: data.site_xpos[ee_frame_id].astype(jp.float32),
			Columns.STATE: jp.concatenate([data.qpos, data.qvel, data.qfrc_actuator]).astype(jp.float32),
			Columns.ACTIONS: action.astype(jp.float32),
		}
		return state.replace(data=data, obs=obs, reward=reward, done=done, info=info)

	def set_target(self, target: np.ndarray) -> None:
		"""Set the target position used on the next reset()."""
		self._target_pos = np.asarray(target, dtype=np.float32)

	def getJointLimits(self):
		joint_range = np.asarray(self._mj_model.jnt_range, dtype=np.float32)
		nj = int(self._mj_model.njnt)
		if joint_range.shape[0] != nj or not np.any(joint_range):
			return -np.pi * np.ones(nj, dtype=np.float32), np.pi * np.ones(nj, dtype=np.float32)
		joint_low, joint_high = joint_range[:, 0], joint_range[:, 1]
		if joint_low.shape[0] < nj:
			extra = nj - joint_low.shape[0]
			joint_low = np.concatenate([joint_low, np.repeat(joint_low[-1], extra)])
			joint_high = np.concatenate([joint_high, np.repeat(joint_high[-1], extra)])
		return joint_low, joint_high

	# --- Helpers ---

	def _get_obs(self, data: mjx.Data, target_pos: jax.Array) -> jax.Array:
		ee_frame_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SITE, "ee_frame")
		ee_pos = data.site_xpos[ee_frame_id]
		proprioception = jp.concatenate([data.qpos, data.qvel, data.qfrc_actuator])
		return jp.concatenate([ee_pos, target_pos, proprioception]).astype(jp.float32)

	def _get_reward(self, obs: jax.Array) -> jax.Array:
		distance = jp.linalg.norm(obs[:3] - obs[3:6])
		return -distance


def make_env(spec: mujoco.MjSpec, **kwargs) -> "AbstractMjxEnv":
	"""Create an AbstractMjxEnv from a mujoco.MjSpec.

	Args:
		spec: A MjSpec describing the robot.
		**kwargs: Config overrides (e.g. ctrl_dt=0.02, sim_dt=0.005).
	"""
	return AbstractMjxEnv(spec, config_overrides=kwargs or None)
