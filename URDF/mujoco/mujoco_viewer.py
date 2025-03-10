import mujoco
import mujoco.viewer
import sys

if (len(sys.argv)) != 2:
    print("Usage: python3 mujoco_viewer <XML file>")
    exit()

# Load the XML model
model = mujoco.MjModel.from_xml_path(sys.argv[1])
data = mujoco.MjData(model)

# Actuators
actuators = ['index_mcp_actuator', '_index_pip_actuator', 'middle_mcp_actuator',
            '_middle_pip_actuator', 'ring_mcp_actuator', '_ring_pip_actuator',
            'pinky_mcp_actuator', '_pinky_pip_actuator', 'thumb_flexor_actuato',
            'thumb_rotator_actuator']

# Create a viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # Mimic Joints
        act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in actuators]
        for j in range(0,7,2):
            if act_ids[j] != -1 and act_ids[j+1] != -1:
                data.ctrl[j+1] = data.ctrl[j] * 1.05851325 + 0.72349796

        mujoco.mj_step(model, data)
        viewer.sync()
