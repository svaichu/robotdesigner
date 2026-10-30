import numpy as np
import pytest

from robotdesigner.config import ConfigError, Pose, chain
from robotdesigner.config import kinematics as kin

COMMON = dict(radius=0.03, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0)
PI = np.pi


def ee(arm, q):
    return kin.forward(arm, np.asarray(q, dtype=float)).ee[:3, 3]


def test_worked_examples():
    zz = chain(2, "zz", link_length=0.5, mass=1.0, **COMMON)
    assert np.allclose(ee(zz, (0, 0)), (1.0, 0, 0), atol=1e-12)
    assert np.allclose(ee(zz, (PI / 2, 0)), (0, 1.0, 0), atol=1e-12)
    assert np.allclose(ee(zz, (PI / 2, PI / 2)), (-0.5, 0.5, 0), atol=1e-12)
    zy = chain(2, "zy", link_length=0.5, mass=1.0, **COMMON)
    assert np.allclose(ee(zy, (0, PI / 2)), (0.5, 0, -0.5), atol=1e-12)


def test_base_translation_shifts_ee():
    a = chain(2, "zz", link_length=0.5, mass=1.0, **COMMON)
    q = (0.3, -0.7)
    mount = Pose((0, 0, 0.1), (1, 0, 0, 0))
    shifted = kin.forward(a, q, mount).ee[:3, 3]
    assert np.allclose(shifted - ee(a, q), (0, 0, 0.1), atol=1e-12)
    # the world-frame end effector is the mount applied to the base-frame one
    T = kin.pose_to_matrix(Pose((0.2, -0.1, 0.4), (0.8, 0.2, 0.4, 0.1)))
    world = kin.forward(a, q, Pose((0.2, -0.1, 0.4), (0.8, 0.2, 0.4, 0.1))).ee
    assert np.allclose(world, T @ kin.forward(a, q).ee, atol=1e-12)


def test_batch_matches_single():
    arm = chain(4, "zyyz", total_length=1.0, total_mass=2.0, range=(-2.0, 2.0), **COMMON)
    q = kin.sample_joint_positions(arm, 7, np.random.default_rng(0))
    batch = kin.forward_ee_batch(arm, q)
    assert batch.shape == (7, 3)
    for row, qi in zip(batch, q):
        assert np.allclose(row, ee(arm, qi), atol=1e-12)
    fk = kin.forward(arm, q[0])
    assert fk.link_frames.shape == (4, 4, 4) and fk.tip_frames.shape == (4, 4, 4) and fk.ee.shape == (4, 4)
    assert np.allclose(fk.tip_frames[-1], fk.ee)


def test_sample_respects_ranges():
    arm = chain(2, "zy", link_length=0.5, mass=1.0, range=[None, (-0.1, 0.1)], **COMMON)
    q = kin.sample_joint_positions(arm, 500, np.random.default_rng(1))
    assert np.all(np.abs(q[:, 0]) <= PI) and np.all(np.abs(q[:, 1]) <= 0.1)


@pytest.mark.parametrize(
    "kwargs",
    [
        dict(axes="zy", link_length=0.5, total_length=1.0, mass=1.0),
        dict(axes="zy", link_length=0.5, mass=None),
        dict(axes="z", link_length=0.5, mass=1.0),
        dict(axes="zq", link_length=0.5, mass=1.0),
    ],
)
def test_argument_validation(kwargs):
    with pytest.raises(ConfigError):
        chain(2, **kwargs, **COMMON)


def test_uniform_splits():
    arm = chain(3, "zyz", total_length=0.9, total_mass=3.0, **COMMON)
    assert sum(s.link.tip_offset[0] for s in arm.segments) == pytest.approx(0.9)
    assert sum(s.link.mass for s in arm.segments) == pytest.approx(3.0)
    assert arm.config_hash == chain(3, "zyz", total_length=0.9, total_mass=3.0, **COMMON).config_hash


def test_explicit_sequences_override():
    arm = chain(2, "zy", link_length=[0.2, 0.7], mass=[1.0, 2.0], **COMMON)
    assert [s.link.tip_offset[0] for s in arm.segments] == [0.2, 0.7]
    with pytest.raises(ConfigError):
        chain(2, "zy", link_length=[0.2], mass=1.0, **COMMON)
