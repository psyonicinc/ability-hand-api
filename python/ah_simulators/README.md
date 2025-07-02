# Ability Hand Simulators

## Mujoco 

<div style="text-align: center;">
    <img src="./images/unitree_z1.png" alt="Ability Hand on Unitree Z1" width="400" />
</div>
  

Included in the API are a collection of Mujoco hand descriptions (large, small, 
left and right) and scenes in XML format.  Collisions are detected using a 
simplified version of meshes of the hand and inertia is calcuated by applying 
the known masses to the simplified meshes.  The touch sensors of the Ability 
Hand are also simulated. 

### Mujoco Examples

To launch the hand alone use

```python3 ./mujoco_xml/mujoco_viewer.py ./mujoco_xml/scene.xml```

Examples of popular robots have been integrated from Mujoco's [menagerie](https://github.com/google-deepmind/mujoco_menagerie) 
:

- Unitree Z1 : ```python3 ./mujoco_xml/mujoco_viewer.py ./mujoco_xml/unitree_z1/scene.xml```
- Franka FR3 : ```python3 ./mujoco_xml/mujoco_viewer.py ./mujoco_xml/franka_fr3/scene.xml```
- UFACTORY xArm7 : ```python3 ./mujoco_xml/mujoco_viewer.py ./mujoco_xml/ufactory_xarm7/scene.xml```
- Unitree G1 : ```python3 ./mujoco_xml/mujoco_viewer.py ./mujoco_xml/unitree_g1/scene.xml```
- Unitree H1 : ```python3 ./mujoco_xml/mujoco_viewer.py ./mujoco_xml/unitree_h1/scene.xml```
- UR5e : ```python3 ./mujoco_xml/mujoco_viewer.py ./mujoco_xml/universal_robots_ur5e/scene.xml```
- UR10e : ```python3 ./mujoco_xml/mujoco_viewer.py ./mujoco_xml/universal_robots_ur10e/scene.xml```

### Integrating with Python Wrapper

Navigate to the ah_simulators directory in the python folder and install the necessary requirements for the Mujoco simulation, preferably in a virtual environment. 

```python

python3 -m pip install -r requirements.txt

```
 Return to the root python directory and open a python terminal. If using Windows or Linux you can use a traditional python terminal such as, 

```python
python3
```
If using a MacOS use, 


```python
mjpython
```

You can control the mujoco hand using the same serial client as the real 
hand and make the serial connection virtual by passing the `simulated=True`
argument. Create a AHMujocoSim class and pass in the Hand class and select a scene.

```python

from ah_wrapper.ah_serial_client import AHSerialClient
from ah_simulators.ah_mujoco import AHMujocoSim

client = AHSerialClient(simulated=True, write_thread="False")
sim = AHMujocoSim(hand=client.hand, scene='ah_simulators/mujoco_xml/unitree_g1/scene.xml')
client.set_position(90)
```

You can also pass a non-simulated client's hand to the simulator and use it to
visually represent what the hand is doing.

The mujoco simulator reads the targets from the hand class and applies them to 
the joints in the simulator.  At this time only position control works, but 
velocity, torque and grip control are on the roadmap to be added.

You can also run the simulated hand wave example in the root python directory

```python3 simulated_hand_wave.py```

This is also a great example for how to control two hands at the same time.

### Simulating Four Bar Linkage

The hand is a 4 bar linkage mechanism forming an 'X'. 4 bar linkages are not 
supported in the urdf/mujoco format. The motion of the finger pip joint is based on 
a approximate linear relationship to the mcp joint.  This 'mimicing' is done via
python loop code rather than the equality property in the xml.  For Example:

```python
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

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        for j in range(0, 7, 2):
            data.ctrl[act_ids[j + 1]] = (
                data.ctrl[act_ids[j]] * 1.05851325 + 0.72349796
            )
        mujoco.mj_step(model, data)
        viewer.sync()
```

## Issac Simulator (4.5)

You can import the URDF from the [ROS2 package](https://github.com/psyonicinc/ability-hand-ros2/tree/main/src/ah_urdf/urdf)
directly using  

File ➡️ Import

And selecting the URDF

Select the following options:

- Options ➡️ Create in Stage
- Moveable Base
- Set all _q2 mimic joints Natural Frequency to 0.0 
- Collision From Visuals
- Allow Self Collisions

Click Import
