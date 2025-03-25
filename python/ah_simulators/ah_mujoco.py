import threading

import mujoco
import mujoco.viewer
from math import radians

from ah_wrapper.hand import Hand


class Controller:
    def __init__(self, hand: Hand, model, left_hand=False):
        self.hand = hand
        self.actuators = [
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
        if left_hand:
            self.actuators = ["l_" + i for i in self.actuators]

        self.act_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in self.actuators
        ]
        if -1 in self.act_ids:
            print("Cannot find all AH actuators in scene")
            
        self.controllable_ids = [self.act_ids[i] for i in range(0, 7, 2)]
        self.controllable_ids.append(self.act_ids[-2])
        self.controllable_ids.append(self.act_ids[-1])
        
        self.Kp = 10
        self.Kv = 0.09
        
    def actuate(self, data):
        current_position = [data.qpos[i] for i in self.controllable_ids]
        current_velocity = [data.qvel[i] for i in self.controllable_ids]
        # TODO some reason qpos for rotator and flexor are swapped but data.ctrl is not
        current_position[-1], current_position[-2] = current_position[-2], current_position[-1]
        current_velocity[-1], current_velocity[-2] = current_velocity[-2], current_velocity[-1]
        target_position = self.hand.get_tar_position()
        target_velocity = self.hand.get_tar_velocity()
        
        for i in range(6):
            if target_position:
                # position_error =  radians(target_position[i]) - current_position[i]
                # control_signal = self.Kp * position_error
                # TODO I imagine the built in position controller is conflicting with this controller, revisit in future
                # self.data.ctrl[self.controllable_ids[i]] = control_signal
                # TODO just set to target position always instead for now
                data.ctrl[self.controllable_ids[i]] = radians(
                    target_position[i]
                )
            elif target_velocity:
                # velocity_error = radians(target_velocity[i]) - current_velocity[i]
                # control_signal = self.Kv * velocity_error
                pass

        self.mimic_joints(data)
    
    def mimic_joints(self, data):
        for j in range(0, 7, 2):
            data.ctrl[self.act_ids[j + 1]] = (
                    data.ctrl[self.act_ids[j]] * 1.05851325
                    + 0.72349796
            )

class AHMujocoSim:
    def __init__(self, scene: str, hand: Hand, left_hand = None):
        self.mujuco_thread = threading.Thread(target=self.mujoco_loop)

        # Load the XML model
        self.model = mujoco.MjModel.from_xml_path(scene)
        self.controller = Controller(hand=hand, model=self.model)
        if left_hand:
            self.l_controller = Controller(hand=left_hand, left_hand=True, model=self.model)
        else:
            self.l_controller = None
        self.data = mujoco.MjData(self.model)
        self.mujuco_thread.start()

    def mujoco_loop(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                self.controller.actuate(data=self.data)
                if self.l_controller:
                    self.l_controller.actuate(data=self.data)
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
