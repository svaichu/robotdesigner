from pathlib import Path
import mujoco

xml = Path(__file__).parent / "elrobot_table.xml"
model = mujoco.MjModel.from_xml_path(str(xml))
data = mujoco.MjData(model)

if __name__ == "__main__":
    import mujoco.viewer
    mujoco.viewer.launch(model, data)
