"""The status table of docs/robots/menagerie.md is the one in tests/menagerie_expected.json (no library needed)."""

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROW = re.compile(r"^\| `([^`]+)` \| [^|]+ \| [^|]+ \| ([^|]+) \|$", re.M)


def test_the_docs_table_lists_the_expected_status_of_every_robot():
    expected = json.loads((ROOT / "tests" / "menagerie_expected.json").read_text())
    rows = {name: status.strip() for name, status in ROW.findall((ROOT / "docs" / "robots" / "menagerie.md").read_text())}
    assert rows == expected, "docs/robots/menagerie.md and tests/menagerie_expected.json disagree"


def test_the_counts_in_the_prose_are_the_counts_of_the_file():
    expected = json.loads((ROOT / "tests" / "menagerie_expected.json").read_text())
    ok = sum(v == "ok" for v in expected.values())
    assert f"Today {ok} run\nand {len(expected) - ok} are refused" in (ROOT / "docs" / "robots" / "menagerie.md").read_text()
