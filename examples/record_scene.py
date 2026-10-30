"""Record a random-policy rollout in the robosuite Lift scene as MP4 and GIF, all envs tiled and labelled.

Run (headless):  MUJOCO_GL=egl python examples/record_scene.py [out_dir]
Needs the robosuite and viz extras:  pip install -e ".[env,robosuite,viz]"
"""

from __future__ import annotations

import sys
from pathlib import Path

from robosuite_scene import build_config

from robotdesigner.envs import make_env
from robotdesigner.viz import record_rollout


def main(out_dir: str = ".", steps: int = 150, num_envs: int = 4) -> None:
    env = make_env(build_config(num_envs=num_envs, horizon=steps))
    # camera "rs_sideview" is a camera of the robosuite arena; every 2nd step is 25 fps, an exact GIF rate
    traj, clip = record_rollout(env, steps, env_index="all", every=2, camera="rs_sideview")
    print(clip)
    out = Path(out_dir)
    print("saved", clip.save(out / "scene.mp4"), clip.save(out / "scene.gif"))
    env.close()


if __name__ == "__main__":
    main(*sys.argv[1:2])
