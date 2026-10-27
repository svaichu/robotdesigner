import subprocess
import sys

BLOCKED = ["torch", "torchrl", "tensordict", "mujoco", "mujoco_warp", "robosuite"]


def test_config_import_has_no_engine():
    code = (
        "import sys\n"
        f"for m in {BLOCKED!r}: sys.modules[m] = None\n"
        "import robotdesigner.config\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
