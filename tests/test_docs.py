"""The documentation is checked against the code.

* Python blocks in the configuration pages and the index are executed (blocks fenced as ``python no-run`` are not):
  the pages use only ``robotdesigner.config`` (numpy), so this needs no simulator.
* Every relative link in the Markdown files points to an existing file and, if it has a fragment, to a real heading.
"""

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
RUNNABLE = sorted((ROOT / "docs" / "configs").glob("*.md")) + [ROOT / "docs" / "README.md"]
ALL_MD = [ROOT / "README.md"] + sorted((ROOT / "docs").rglob("*.md"))
FENCE = re.compile(r"^```python\n(.*?)^```", re.S | re.M)
LINK = re.compile(r"(?<!!)\[[^\]]+\]\(([^)\s]+)\)")
HEADING = re.compile(r"^#{1,6}\s+(.*?)\s*$", re.M)


def slug(heading: str) -> str:
    """GitHub's anchor for a heading."""
    return re.sub(r"[^\w\- ]", "", heading.lower()).replace(" ", "-")


def anchors(path: Path) -> set[str]:
    text = re.sub(r"^```.*?^```", "", path.read_text(), flags=re.S | re.M)  # headings inside code blocks do not count
    return {slug(h) for h in HEADING.findall(text)}


def test_pages_exist():
    assert {p.name for p in RUNNABLE} >= {"robot.md", "control.md", "task.md", "sim.md", "env.md", "README.md"}


@pytest.mark.parametrize("page", RUNNABLE, ids=lambda p: f"{p.parent.name}/{p.name}")
def test_documented_examples_run(page):
    blocks = FENCE.findall(page.read_text())
    assert blocks, f"{page.name} has no python example"
    namespace = {"__name__": "doc_example"}
    for i, code in enumerate(blocks):
        exec(compile(code, f"{page.name}[block {i}]", "exec"), namespace)  # noqa: S102


@pytest.mark.parametrize("page", ALL_MD, ids=lambda p: str(p.relative_to(ROOT)))
def test_links_resolve(page):
    text = re.sub(r"^```.*?^```", "", page.read_text(), flags=re.S | re.M)
    for target in LINK.findall(text):
        if re.match(r"^[a-z]+://|^mailto:", target):
            continue
        file_part, _, fragment = target.partition("#")
        dest = page if not file_part else (page.parent / file_part).resolve()
        assert dest.exists(), f"{page.name}: {target} does not exist"
        if fragment and dest.suffix == ".md":
            assert fragment in anchors(dest), f"{page.name}: no heading for #{fragment} in {dest.name}"
