"""Robot embodiments as frozen, validated, hashable data (numpy only; no simulator is imported)."""

from .arm import Capsule, JointSpec, LinkSpec, ParametricArm, Segment, ToolSpec
from .generators import chain
from .gripper import TwoFingerGripper
from .pose import Pose

__all__ = [
    "Capsule", "JointSpec", "LinkSpec", "ParametricArm", "Pose", "Segment", "ToolSpec", "TwoFingerGripper", "chain",
]
