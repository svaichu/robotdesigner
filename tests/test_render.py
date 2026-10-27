import numpy as np
import pytest

pytest.importorskip("torchrl")
mujoco = pytest.importorskip("mujoco")


def _has_gl() -> bool:
    try:
        with mujoco.Renderer(mujoco.MjModel.from_xml_string("<mujoco/>"), 16, 16):
            return True
    except Exception:
        return False


@pytest.mark.render
@pytest.mark.skipif(not _has_gl(), reason="no OpenGL context (try MUJOCO_GL=egl)")
def test_render_rgb_shape_and_dtype():
    from robotdesigner.backends.mujoco.render import render_rgb
    from robotdesigner.envs import make_env
    from tests.test_reach_env import make_cfg

    env = make_env(make_cfg(num_envs=2))
    env.reset()
    frame = render_rgb(env, env_index=1, size=(48, 64))
    assert frame.shape == (48, 64, 3) and frame.dtype == np.uint8


def test_render_rejects_bad_env_index():
    from robotdesigner.backends.mujoco.render import render_rgb
    from robotdesigner.envs import make_env
    from tests.test_reach_env import make_cfg

    env = make_env(make_cfg(num_envs=2))
    with pytest.raises(IndexError):
        render_rgb(env, env_index=5)
