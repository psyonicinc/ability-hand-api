import mujoco
import mujoco.viewer
import sys

if (len(sys.argv)) != 2:
    print(
        "Usage: python3 mujoco_viewer <XML file> this script will automatically map the mimic joints in the ability hand"
    )
    exit()

# Load the XML model
model = mujoco.MjModel.from_xml_path(sys.argv[1])
data = mujoco.MjData(model)

# Actuators
actuators = [
    "index_mcp_actuator",
    "_index_pip_actuator",
    "middle_mcp_actuator",
    "_middle_pip_actuator",
    "ring_mcp_actuator",
    "_ring_pip_actuator",
    "pinky_mcp_actuator",
    "_pinky_pip_actuator",
    "thumb_flexor_actuator",
    "thumb_rotator_actuator",
]
act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in actuators]

# Create a viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        if -1 not in act_ids:
            for j in range(0, 7, 2):
                data.ctrl[act_ids[j + 1]] = (
                    data.ctrl[act_ids[j]] * 1.05851325 + 0.72349796
                )

        mujoco.mj_step(model, data)
        viewer.sync()
