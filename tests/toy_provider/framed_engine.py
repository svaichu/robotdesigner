"""Engine side of ``framed/reach``: ``toy/reach`` plus a mount marker, nested in a static body to show that nesting is read."""

from __future__ import annotations

from .framed import ELSEWHERE, FRAME
from .reach import ReachTask

MARKER = "mount_marker"


class FramedReach(ReachTask):
    AT = FRAME

    def attach(self, spec) -> None:
        holder = spec.worldbody.add_body(name="mount_holder", pos=list(self.AT.pos), quat=list(self.AT.quat))
        holder.add_site(name=MARKER, size=[0.01, 0.01, 0.01])

    def mount_site(self) -> str:
        return MARKER


class MisplacedReach(FramedReach):
    AT = ELSEWHERE
