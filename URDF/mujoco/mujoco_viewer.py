import mujoco
import mujoco.viewer
import sys

if (len(sys.argv)) != 2:
    print("Usage: python3 mujoco_viewer <XML file>")
    exit()

# Load the XML model
model = mujoco.MjModel.from_xml_path(sys.argv[1])
data = mujoco.MjData(model)

# Create a viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
