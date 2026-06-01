import tempfile
from pathlib import Path

import mujoco
import pytest

from robotdesigner.mujoco.composer import build_arm, save_arm


@pytest.mark.parametrize("ndof", [1, 2, 3, 5])
def test_build_arm_returns_mjspec(ndof):
    spec = build_arm(ndof=ndof)
    assert isinstance(spec, mujoco.MjSpec)


@pytest.mark.parametrize("control", ["pos", "torque"])
def test_build_arm_control_modes(control):
    spec = build_arm(ndof=3, control=control)
    model = spec.compile()
    assert model.nu == 3


@pytest.mark.parametrize("ndof", [1, 2, 3, 5])
def test_build_arm_compiles_correct_ndof(ndof):
    spec = build_arm(ndof=ndof)
    model = spec.compile()
    assert model.nv == ndof


def test_build_arm_custom_orientation():
    spec = build_arm(ndof=3, control="pos", joint_orientation=["z", "y", "z"])
    model = spec.compile()
    assert model.nv == 3


def test_build_arm_orientation_cycles():
    # orientation shorter than ndof — should cycle without error
    spec = build_arm(ndof=4, joint_orientation=["z"])
    model = spec.compile()
    assert model.nv == 4


def test_build_arm_model_name():
    spec = build_arm(ndof=3, control="pos")
    assert spec.modelname == "3_arm_pos"


def test_build_arm_ee_frame_site_exists():
    ndof = 3
    spec = build_arm(ndof=ndof)
    model = spec.compile()
    site_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        for i in range(model.nsite)
    ]
    assert any("ee_frame" in name for name in site_names)


def test_build_arm_invalid_ndof():
    with pytest.raises(AssertionError):
        build_arm(ndof=0)


def test_build_arm_invalid_control():
    with pytest.raises(AssertionError):
        build_arm(ndof=3, control="velocity")


def test_save_arm_creates_file():
    spec = build_arm(ndof=3, control="pos", joint_orientation=["z", "y", "z"])
    with tempfile.TemporaryDirectory() as tmpdir:
        path = save_arm(
            spec,
            ndof=3,
            control="pos",
            joint_orientation=["z", "y", "z"],
            output_dir=Path(tmpdir),
        )
        assert path.exists()
        assert path.suffix == ".xml"
        assert "three" in path.name
        assert "pos" in path.name
        assert "zyz" in path.name


def test_save_arm_xml_is_valid():
    spec = build_arm(ndof=2, control="torque")
    with tempfile.TemporaryDirectory() as tmpdir:
        path = save_arm(spec, ndof=2, control="torque", output_dir=Path(tmpdir))
        xml = path.read_text()
        # round-trip: MuJoCo must be able to parse the saved XML
        model = mujoco.MjModel.from_xml_string(xml)
        assert model.nv == 2
