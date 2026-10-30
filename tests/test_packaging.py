import subprocess
import sys

BLOCKED = ["torch", "torchrl", "tensordict", "mujoco", "mujoco_warp", "warp", "robosuite"]


def test_config_import_has_no_engine():
    code = (
        "import sys\n"
        f"for m in {BLOCKED!r}: sys.modules[m] = None\n"
        "import robotdesigner.config\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_viz_import_has_no_engine():
    code = (
        "import sys\n"
        f"for m in {BLOCKED!r}: sys.modules[m] = None\n"
        "import robotdesigner.viz\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_top_level_import_is_lazy_and_exposes_make_env():
    code = (
        "import sys\n"
        f"for m in {BLOCKED!r}: sys.modules[m] = None\n"
        "import robotdesigner\n"
        "assert 'make_env' in dir(robotdesigner) and 'make_env' in robotdesigner.__all__\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
    code = "import robotdesigner, robotdesigner.envs as e; assert robotdesigner.make_env is e.make_env"
    subprocess.run([sys.executable, "-c", code], check=True)


def test_every_provider_package_imports_without_an_engine():
    code = (
        "import sys, pkgutil\n"
        f"for m in {BLOCKED!r}: sys.modules[m] = None\n"
        "import importlib, robotdesigner.tasks as t\n"
        "names = [m.name for m in pkgutil.iter_modules(t.__path__) if m.ispkg]\n"
        "assert names\n"
        "for n in names: importlib.import_module('robotdesigner.tasks.' + n)\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
