import threading

import mujoco
import mujoco.viewer
from math import radians

from ah_wrapper.hand import Hand


class AHMujocoSim:
    def __init__(self, scene: str, hand: Hand, left_hand = None, simulated_feedback=True):
        self.hand = hand
        self.left_hand = left_hand
        self.simulated_feedback = simulated_feedback
        self.mujuco_thread = threading.Thread(target=self.mujoco_loop)

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
        l_actuators = ["l_" + i for i in actuators]

            # Load the XML model
        self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)
        self.act_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in actuators
        ]
        if -1 in self.act_ids:
            print("Cannot find all AH actuators in scene")
            exit(1)
        if left_hand:
            self.l_act_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in l_actuators]
            if -1  in self.l_act_ids:
                print("Cannot find all Left AH actuators in scene")
            self.l_controllable_ids = [self.l_act_ids[i] for i in range(0, 7, 2)]
            self.l_controllable_ids.append(self.l_act_ids[-2])
            self.l_controllable_ids.append(self.l_act_ids[-1])

        self.controllable_ids = [self.act_ids[i] for i in range(0, 7, 2)]
        self.controllable_ids.append(self.act_ids[-2])
        self.controllable_ids.append(self.act_ids[-1])

        # Setup position / velocity controller
        self.Kp = 10
        self.Kv = 0.09

        self.mujuco_thread.start()

    def controller(self):
        current_position = [self.data.qpos[i] for i in self.controllable_ids]
        # TODO some reason qpos for rotator and flexor are swapped but data.ctrl is not
        temp = current_position[-1]
        current_position[-1] = current_position[-2]
        current_position[-2] = temp
        current_velocity = [self.data.qvel[i] for i in self.controllable_ids]
        temp = current_velocity[-1]
        current_velocity[-1] = current_velocity[-2]
        current_velocity[-2] = temp

        target_position = self.hand.get_tar_position()
        target_velocity = self.hand.get_tar_velocity()
        for i in range(6):
            if target_position:
                # position_error =  radians(target_position[i]) - current_position[i]
                # control_signal = self.Kp * position_error
                # TODO I imagine the built in position controller is conflicting with this controller, revisit in future
                # self.data.ctrl[self.controllable_ids[i]] = control_signal
                # TODO just set to target position always instead for now
                self.data.ctrl[self.controllable_ids[i]] = radians(
                    target_position[i]
                )
            elif target_velocity:
                # velocity_error = radians(target_velocity[i]) - current_velocity[i]
                # control_signal = self.Kv * velocity_error
                pass

    def mujoco_loop(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                self.controller()

                # Mimic L1 joints -> L2 joints
                for j in range(0, 7, 2):
                    self.data.ctrl[self.act_ids[j + 1]] = (
                        self.data.ctrl[self.act_ids[j]] * 1.05851325
                        + 0.72349796
                    )

                if self.left_hand:
                    for j in range(0, 7, 2):
                        self.data.ctrl[self.l_act_ids[j + 1]] = (
                                self.data.ctrl[self.l_act_ids[j]] * 1.05851325
                                + 0.72349796
                        )

                mujoco.mj_step(self.model, self.data)
                viewer.sync()
