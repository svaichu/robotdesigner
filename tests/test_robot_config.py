import pytest

from robotdesigner.config import (
    Capsule,
    ConfigError,
    JointSpec,
    LinkSpec,
    ParametricArm,
    Pose,
    Segment,
)


def make_arm(axis0=(0, 0, 1), mass0=1.0, range1=(-1.8, 1.8), home=(0.0, 0.0), label="two-link demo", neg_zero=False):
    return ParametricArm(
        schema_version=1,
        segments=(
            Segment(
                joint=JointSpec(axis=axis0, range=None, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=50.0),
                link=LinkSpec(
                    joint_frame=Pose.identity(),
                    tip_offset=(0.4, -0.0 if neg_zero else 0.0, 0.0),
                    mass=mass0,
                    geometry=Capsule(radius=0.03),
                    inertia=None,
                ),
            ),
            Segment(
                joint=JointSpec(axis=(0, 1, 0), range=range1, damping=0.5, armature=0.01, frictionloss=0.0, effort_limit=30.0),
                link=LinkSpec(
                    joint_frame=Pose.identity(),
                    tip_offset=(0.3, 0.0, 0.0),
                    mass=0.8,
                    geometry=Capsule(radius=0.025),
                    inertia=None,
                ),
            ),
        ),
        home_qpos=home,
        end_effector=Pose.identity(),
        tool=None,
        label=label,
    )


def test_construction_and_round_trip():
    arm = make_arm()
    assert arm.n == 2
    assert len(arm.config_hash) == 12 and arm.full_hash.startswith(arm.config_hash)
    back = ParametricArm.from_dict(arm.to_dict())
    assert back == arm and back.full_hash == arm.full_hash
    assert arm.canonical_json() == back.canonical_json()


def test_frozen_and_tuples():
    arm = make_arm()
    with pytest.raises(AttributeError):
        arm.label = "x"
    assert isinstance(arm.segments, tuple) and isinstance(arm.home_qpos, tuple)


@pytest.mark.parametrize(
    "build, path",
    [
        (lambda: make_arm(axis0=(0, 0, 0)), "axis"),
        (lambda: make_arm(range1=(1.0, 1.0)), "range"),
        (lambda: make_arm(mass0=0.0), "mass"),
        (lambda: make_arm(home=(0.0, 2.0)), "home_qpos[1]"),
        (lambda: make_arm(home=(0.0,)), "home_qpos"),
    ],
)
def test_invalid_inputs_raise_with_path(build, path):
    with pytest.raises(ConfigError) as e:
        build()
    assert str(e.value).startswith(path)


def test_from_dict_prefixes_full_path():
    d = make_arm().to_dict()
    d["segments"][1]["joint"]["range"] = [1.0, 0.0]
    with pytest.raises(ConfigError, match=r"^segments\[1\]\.joint\.range: lo must be < hi"):
        ParametricArm.from_dict(d)


def test_from_dict_rejects_unknown_keys_and_bad_schema():
    d = make_arm().to_dict()
    d["extra"] = 1
    with pytest.raises(ConfigError, match="unknown keys: extra"):
        ParametricArm.from_dict(d)
    d = make_arm().to_dict()
    d["segments"][0]["link"]["bogus"] = 1
    with pytest.raises(ConfigError, match=r"^segments\[0\]\.link: unknown keys"):
        ParametricArm.from_dict(d)
    d = make_arm().to_dict()
    d["schema_version"] = 99
    with pytest.raises(ConfigError, match="^schema_version"):
        ParametricArm.from_dict(d)


def test_hash_invariants():
    base = make_arm()
    assert make_arm(axis0=(0, 0, 2)).full_hash == base.full_hash
    assert make_arm(neg_zero=True).full_hash == base.full_hash
    assert make_arm(label="other").full_hash == base.full_hash
    assert make_arm(mass0=1.000001).full_hash != base.full_hash
    d = base.to_dict()
    reordered = dict(reversed(list(d.items())))
    assert ParametricArm.from_dict(reordered).full_hash == base.full_hash


def test_quaternion_canonical_sign():
    assert Pose((0, 0, 0), (-2, 0, 0, 0)).quat == (1.0, 0.0, 0.0, 0.0)
    assert Pose((0, 0, 0), (-1, 0, 0, 0)) == Pose.identity()
