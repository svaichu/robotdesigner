"""The examples are notebooks only: valid, free of stored errors, and the ones that replace the old scripts run.

The structure checks need nothing installed. Running a notebook needs its extras: ``pytest -m robosuite`` (CPU), ``-m gpu`` (Warp),
``-m render`` (videos, needs an OpenGL context: MUJOCO_GL=egl).
"""

import json

import pytest

from tests.nbrun import EXAMPLES, run_notebook

NOTEBOOKS = sorted(EXAMPLES.glob("*.ipynb"))


def test_examples_are_notebooks_only():
    assert NOTEBOOKS, "no example notebooks found"
    assert not list(EXAMPLES.glob("*.py")), "examples are notebooks; a script belongs in a notebook cell"


@pytest.mark.parametrize("path", NOTEBOOKS, ids=lambda p: p.name)
def test_notebook_is_valid_and_has_no_stored_error(path):
    nb = json.loads(path.read_text())
    assert nb["nbformat"] == 4
    ids = [c["id"] for c in nb["cells"]]
    assert len(ids) == len(set(ids)), "cell ids must be unique"
    for i, cell in enumerate(nb["cells"]):
        if cell["cell_type"] != "code":
            continue
        compile("".join(cell["source"]), f"{path.name}[cell {i}]", "exec")
        assert not [o for o in cell["outputs"] if o["output_type"] == "error"], f"{path.name} cell {i} stores an error"


@pytest.mark.robosuite
def test_quickstart_cpu_mujoco_notebook_runs():
    pytest.importorskip("robosuite")
    pytest.importorskip("torchrl")
    ns = run_notebook("quickstart_cpu_mujoco.ipynb")
    assert tuple(ns["traj"].shape) == (4, 50)
    assert {"cube_pos", "ee_pos"} <= set(ns["traj"]["next", "task"].keys())


@pytest.mark.gpu
@pytest.mark.robosuite
def test_quickstart_gpu_warp_notebook_runs():
    torch = pytest.importorskip("torch")
    pytest.importorskip("robosuite")
    pytest.importorskip("mujoco_warp")
    if not torch.cuda.is_available():
        pytest.skip("needs a CUDA device")
    ns = run_notebook("quickstart_gpu_warp.ipynb", stop_before="## How fast")   # the timing cell is not a test
    try:
        assert tuple(ns["traj"].shape) == (64, 50)
        assert ns["traj"].device.type == "cuda"
    finally:
        ns["env"].close()


@pytest.mark.render
@pytest.mark.robosuite
def test_scene_viz_notebook_runs():
    pytest.importorskip("robosuite")
    pytest.importorskip("torchrl")
    from tests.test_render import _has_gl

    if not _has_gl():
        pytest.skip("no OpenGL context (try MUJOCO_GL=egl)")
    ns = run_notebook("scene_viz.ipynb")
    assert len(ns["clip_all"]) > 0
