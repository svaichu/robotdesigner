"""The examples are notebooks only: valid, free of stored errors, and the ones that replace the old scripts run.

The structure checks need nothing installed. Running a notebook needs its extras: ``pytest -m robosuite`` (CPU), ``-m gpu`` (Warp),
``-m render`` (videos, needs an OpenGL context: MUJOCO_GL=egl).
"""

import json

import pytest

from tests.nbrun import EXAMPLES, run_notebook

try:
    # Evaluated once, at collection. Probing for an OpenGL context a second time at run time aborts the whole process on a
    # machine without a display (SIGABRT from the GL backend, no exception), which is what failed the robosuite CI job.
    from tests.test_render import needs_gl
except pytest.skip.Exception:  # test_render needs mujoco and torchrl; the structure checks here need nothing
    needs_gl = pytest.mark.skip(reason="needs mujoco and torchrl")

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
@needs_gl
def test_scene_viz_notebook_runs():
    pytest.importorskip("robosuite")
    pytest.importorskip("torchrl")
    ns = run_notebook("scene_viz.ipynb")
    assert len(ns["clip_all"]) > 0


# --- what a stored notebook output must not contain ---------------------------------------------------------------

_LEAKS = __import__("re").compile(r"(/work/|/home/|/Users/|/root/|/tmp/|/scratch|[A-Za-z]:\\\\Users)")


@pytest.mark.parametrize("path", NOTEBOOKS, ids=lambda p: p.name)
def test_stored_outputs_hold_no_absolute_path(path):
    """The repository is public: the venv path of the machine a notebook ran on is replaced by ``<site-packages>``."""
    for i, cell in enumerate(json.loads(path.read_text())["cells"]):
        for out in cell.get("outputs", []):
            text = "".join(out.get("text", "")) + "".join(
                v if isinstance(v, str) else "".join(v) for k, v in out.get("data", {}).items() if k.startswith("text/")
            )
            assert not _LEAKS.search(text), f"{path.name} cell {i} stores a path: {_LEAKS.search(text).group(0)!r}"


# --- the Menagerie notebook (marker menagerie; the frames also need an OpenGL context) -------------------------------------


@pytest.mark.menagerie
def test_menagerie_notebook_runs_up_to_the_frames():
    pytest.importorskip("mujoco_menagerie")
    pytest.importorskip("torchrl")
    ns = run_notebook("menagerie_robots.ipynb", stop_before="## 4. A frame of each")
    assert ns["panda"].n == 7 and ns["go1"].n == 12 and set(ns["envs"]) == {"turned 90 degrees, shifted", "0.3 m up"}
    a = ns["envs"]["0.3 m up"].interface.root_qpos_adr
    assert abs(ns["envs"]["0.3 m up"]._sim.data[0].qpos[a + 2] - (ns["go1"].home_root.pos[2] + 0.3)) < 1e-9


@pytest.mark.menagerie
@pytest.mark.render
@needs_gl
def test_menagerie_notebook_renders_its_frames():
    pytest.importorskip("mujoco_menagerie")
    pytest.importorskip("torchrl")
    ns = run_notebook("menagerie_robots.ipynb")
    assert set(ns["frames"]) == {"panda", "go1 after the drop", "go1 turned and shifted"}
    for name, frame in ns["frames"].items():
        assert frame.shape == (240, 320, 3) and frame.std() > 5, f"{name}: the frame is blank"
