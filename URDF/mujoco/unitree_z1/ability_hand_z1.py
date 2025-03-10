import mujoco
import mujoco.viewer
import numpy as np

# Load the XML model
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

# Open a viewer to visualize
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        data.ctrl[7] = (data.ctrl[6] + 0.72349) * 1.05851
        data.ctrl[9] = (data.ctrl[8] + 0.72349) * 1.05851
        data.ctrl[11] = (data.ctrl[10] + 0.72349) * 1.05851
        data.ctrl[13] = (data.ctrl[12] + 0.72349) * 1.05851
        viewer.sync()

