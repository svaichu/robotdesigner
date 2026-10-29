"""Record a random-policy rollout of the native reach task as MP4 and GIF, all envs tiled and labelled.

Run (headless):  MUJOCO_GL=egl python examples/reach_record.py [out_dir]
Needs the viz extra:  pip install -e ".[env,viz]"
"""

from __future__ import annotations

import sys
from pathlib import Path

from reach_cpu import build_config

from robotdesigner.envs import make_env
from robotdesigner.viz import record_rollout


def main(out_dir: str = ".", steps: int = 150) -> None:
    env = make_env(build_config(num_envs=4, horizon=steps))
    traj, clip = record_rollout(env, steps, env_index="all", every=2)  # every 2nd step: 25 fps, an exact GIF rate
    print(clip)
    out = Path(out_dir)
    print("saved", clip.save(out / "reach.mp4"), clip.save(out / "reach.gif"))
    env.close()


if __name__ == "__main__":
    main(*sys.argv[1:2])
