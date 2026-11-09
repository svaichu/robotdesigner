import pytest

mm = pytest.importorskip("mujoco_menagerie")
pytest.importorskip("mujoco")

pytestmark = pytest.mark.menagerie


def test_panda_model_is_available():
    robot = mm.get("franka_emika_panda")
    assert isinstance(robot.oid, str) and robot.oid
    spec = robot.spec("panda")
    assert spec.compile() is not None
