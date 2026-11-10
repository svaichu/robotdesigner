"""Two test-only providers whose scene has a mount frame that is not the identity and a marker site for it.

``framed/reach`` is consistent: ``mount_frame`` and the marker are at ``FRAME``. ``framed_bad/reach`` is not: the provider
says ``FRAME`` and the marker sits at ``ELSEWHERE``, so the compile must refuse it. Config side only; the engine side is
``framed_engine``.
"""

from __future__ import annotations

import math
from typing import Any, ClassVar

from robotdesigner.config import Pose

from . import ReachParams, ToyTasks

__all__ = ["BAD_PROVIDER", "ELSEWHERE", "FRAME", "PROVIDER", "BadParams", "FramedParams"]

FRAME = Pose((0.3, -0.2, 0.1), (math.cos(math.pi / 8), 0.0, 0.0, math.sin(math.pi / 8)))  # yawed 45 degrees
ELSEWHERE = Pose((0.3, -0.2, 0.4), (1.0, 0.0, 0.0, 0.0))


class FramedParams(ReachParams):
    provider: ClassVar[str] = "framed"


class BadParams(ReachParams):
    provider: ClassVar[str] = "framed_bad"


class FramedTasks(ToyTasks):
    PARAMS: type = FramedParams
    ENGINE = "tests.toy_provider.framed_engine:FramedReach"

    def __init__(self):
        self.TASKS = {"reach": (self.PARAMS, self.ENGINE)}

    def mount_frame(self, params: Any) -> Pose:
        return FRAME


class BadTasks(FramedTasks):
    PARAMS = BadParams
    ENGINE = "tests.toy_provider.framed_engine:MisplacedReach"


PROVIDER = FramedTasks()
BAD_PROVIDER = BadTasks()
