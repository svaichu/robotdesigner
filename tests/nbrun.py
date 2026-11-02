"""Run the code cells of an example notebook in-process: a smoke test that needs no Jupyter kernel."""

import json
import pathlib

EXAMPLES = pathlib.Path(__file__).parent.parent / "examples"


def code_cells(name: str, stop_before: str | None = None) -> list[str]:
    """Sources of the code cells of ``examples/<name>``, up to the first markdown cell that contains ``stop_before``."""
    cells = []
    for cell in json.loads((EXAMPLES / name).read_text())["cells"]:
        src = "".join(cell["source"])
        if stop_before and cell["cell_type"] == "markdown" and stop_before in src:
            break
        if cell["cell_type"] == "code":
            cells.append(src)
    return cells


def run_notebook(name: str, stop_before: str | None = None) -> dict:
    """Execute the code cells in one namespace, in order, and return it."""
    ns: dict = {"__name__": "__main__"}
    for i, src in enumerate(code_cells(name, stop_before)):
        exec(compile(src, f"{name}[code cell {i}]", "exec"), ns)
    return ns
