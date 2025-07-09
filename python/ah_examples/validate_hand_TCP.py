from ah_wrapper import AHSerialClient
import time
import numpy as np
import matplotlib.pyplot as plt
import socket
import json
import threading
from typing import Optional, Dict, Any

SLEEP_TIME = 0.01
TCP_PORT = 12345  # You can change this to any secure port


class TCPValidationServer:
    def __init__(self, port: int = TCP_PORT):
        self.port = port
        self.socket = None
        self.connection = None
        self.client_address = None
        self.running = False
        
    def start_server(self):
        """Start the TCP server and wait for connections"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', self.port))
        self.socket.listen(1)
        print(f"TCP Server started on port {self.port}")
        print("Waiting for client connection...")
        
        self.connection, self.client_address = self.socket.accept()
        print(f"Connected to {self.client_address}")
        self.running = True
        
    def send_response(self, message: str):
        """Send a response to the TCP client"""
        if self.connection:
            try:
                self.connection.send((message + "\n").encode())
            except Exception as e:
                print(f"Error sending response: {e}")
                
    def send_data(self, data: Dict[str, Any]):
        """Send JSON data to the TCP client"""
        if self.connection:
            try:
                json_data = json.dumps(data)
                self.connection.send((json_data + "\n").encode())
            except Exception as e:
                print(f"Error sending data: {e}")
                
    def receive_command(self) -> Optional[str]:
        """Receive a command from the TCP client"""
        if self.connection:
            try:
                data = self.connection.recv(1024).decode().strip()
                return data
            except Exception as e:
                print(f"Error receiving command: {e}")
                return None
        return None
        
    def wait_for_command(self, expected_commands: Optional[list] = None) -> str:
        """Wait for a specific command or any command"""
        while self.running:
            command = self.receive_command()
            if command:
                if command == "quit":
                    self.send_response("Shutting down...")
                    self.running = False
                    return "quit"
                elif expected_commands is None or command in expected_commands:
                    self.send_response("ack")
                    return command
                else:
                    self.send_response("invalid")
            time.sleep(0.01)
        return ""
        
    def close(self):
        """Close the TCP connection"""
        self.running = False
        if self.connection:
            self.connection.close()
        if self.socket:
            self.socket.close()


# Global TCP server instance
tcp_server = TCPValidationServer()


def get_average_finger_positions(positions: list):
    return np.mean(positions[0:4])


def validate_velocity():
    """Validate that velocity mode is uniformly moving hands, feedback is correct
    and current draw among 4 fingers is roughly uniform.
    """
    tcp_server.send_response("Starting velocity validation test")
    command = tcp_server.wait_for_command(["start", "skip", "quit"])
    
    if command == "skip":
        tcp_server.send_response("Skipping velocity validation")
        return "skipped"
    elif command == "quit":
        return "quit"
        
    client = AHSerialClient()
    START_POS = 10
    END_POS = 90
    client.set_position(
        [START_POS, START_POS, START_POS, START_POS, START_POS, -50],
        reply_mode=2,
    )
    time.sleep(1)
    colors = ["r", "g", "b", "m"]
    fingers = ("index", "middle", "ring", "pinky")

    t_velocities = [10, 25, 200, 500]
    for vel in t_velocities:
        tcp_server.send_response(f"Testing velocity {vel}")
        
        positions = [client.hand.get_position()]
        currents = [client.hand.get_current()]
        velocities = [client.hand.get_velocity()]
        target_velocities = [[0, 0, 0, 0, 0, 0]]
        timestamps = [time.time()]

        start_time = time.time()
        while get_average_finger_positions(positions[-1]) < 90:
            client.set_velocity([vel, vel, vel, vel, 0, 0], reply_mode=2)
            positions.append(client.hand.get_position())
            currents.append(client.hand.get_current())
            velocities.append(client.hand.get_velocity())
            target_velocities.append(client.hand.get_tar_velocity())
            timestamps.append(time.time())
            time.sleep(SLEEP_TIME)
        client.set_velocity(velocities=0, reply_mode=2)

        while get_average_finger_positions(positions[-1]) > 10:
            client.set_velocity([-vel, -vel, -vel, -vel, 0, 0], reply_mode=2)
            positions.append(client.hand.get_position())
            currents.append(client.hand.get_current())
            velocities.append(client.hand.get_velocity())
            target_velocities.append(client.hand.get_tar_velocity())
            timestamps.append(time.time())
            time.sleep(SLEEP_TIME)
        client.set_velocity(velocities=0, reply_mode=2)
        time_string = f"Total Time: {time.time()-start_time:.2f} Target Time: {(END_POS-START_POS)*2/vel:.2f}"

        # Send data over TCP
        test_data = {
            "test_type": "velocity_validation",
            "target_velocity": vel,
            "positions": positions,
            "currents": currents,
            "velocities": velocities,
            "target_velocities": target_velocities,
            "timestamps": timestamps,
            "time_string": time_string
        }
        tcp_server.send_data(test_data)

        fig, axs = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
        titles = (
            "Position",
            "Current",
            "Velocity",
            "Target - Actual Velocity",
        )
        data = (
            positions,
            currents,
            velocities,
            list((np.array(target_velocities) - np.array(velocities))),
        )

        for i in range(4):
            values = list(zip(*data[i]))
            for j in range(4):
                axs[i].plot(
                    timestamps, values[j], color=colors[j], label=fingers[j]
                )
            axs[i].set_ylabel(titles[i])
            if i == 0:
                axs[i].legend()
            axs[i].grid(True)

        axs[3].set_xlabel("Timestamp")
        plt.suptitle(
            "VELOCITY MODE \n"
            + f"Positions, Currents, Velocities Vs. Time : Target Velocity = {vel} dps"
            + "\n"
            + time_string
        )
        plt.tight_layout()
        plt.show()

        # Thumb flexor test
        tcp_server.send_response("Testing thumb flexor")
        positions = [client.hand.get_position()]
        currents = [client.hand.get_current()]
        velocities = [client.hand.get_velocity()]
        target_velocities = [[0, 0, 0, 0, 0, 0]]
        timestamps = [time.time()]

        start_time = time.time()
        while positions[-1][4] < 90:
            client.set_velocity([0, 0, 0, 0, vel, 0], reply_mode=2)
            positions.append(client.hand.get_position())
            currents.append(client.hand.get_current())
            velocities.append(client.hand.get_velocity())
            target_velocities.append(client.hand.get_tar_velocity())
            timestamps.append(time.time())
            time.sleep(SLEEP_TIME)
        client.set_velocity(velocities=0, reply_mode=2)

        while positions[-1][4] > 10:
            client.set_velocity([0, 0, 0, 0, -vel, 0], reply_mode=2)
            positions.append(client.hand.get_position())
            currents.append(client.hand.get_current())
            velocities.append(client.hand.get_velocity())
            target_velocities.append(client.hand.get_tar_velocity())
            timestamps.append(time.time())
            time.sleep(SLEEP_TIME)
        client.set_velocity(velocities=0, reply_mode=2)
        time_string = f"Total Time: {time.time()-start_time:.2f} Target Time: {(END_POS-START_POS)*2/vel:.2f}"

        # Send thumb data over TCP
        thumb_data = {
            "test_type": "velocity_validation_thumb",
            "target_velocity": vel,
            "positions": positions,
            "currents": currents,
            "velocities": velocities,
            "target_velocities": target_velocities,
            "timestamps": timestamps,
            "time_string": time_string
        }
        tcp_server.send_data(thumb_data)

        fig, axs = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
        titles = (
            "Position",
            "Current",
            "Velocity",
            "Target - Actual Velocity",
        )
        data = (
            positions,
            currents,
            velocities,
            list((np.array(target_velocities) - np.array(velocities))),
        )

        for i in range(4):
            values = list(zip(*data[i]))
            axs[i].plot(timestamps, values[4], label="flexor")
            axs[i].set_ylabel(titles[i])
            if i == 0:
                axs[i].legend()
            axs[i].grid(True)

        axs[3].set_xlabel("Timestamp")
        plt.suptitle(
            "VELOCITY MODE \n"
            + f"Positions, Currents, Velocities Vs. Time : Target Velocity = {vel} dps"
            + "\n"
            + time_string
        )
        plt.tight_layout()
        plt.show()
    client.close()
    tcp_server.send_response("Velocity validation complete")
    return "complete"


def validate_position():
    """Validate that position mode is uniformly moving hands, feedback is correct
    and current draw among 4 fingers is roughly uniform.
    """
    tcp_server.send_response("Starting position validation test")
    command = tcp_server.wait_for_command(["start", "skip", "quit"])
    
    if command == "skip":
        tcp_server.send_response("Skipping position validation")
        return "skipped"
    elif command == "quit":
        return "quit"
        
    client = AHSerialClient()
    START_POS = 10
    END_POS = 90
    client.set_position(
        [START_POS, START_POS, START_POS, START_POS, START_POS, -START_POS],
        reply_mode=2,
    )
    time.sleep(1)
    colors = ["r", "g", "b", "m"]
    fingers = ("index", "middle", "ring", "pinky")

    t_velocities = [10, 25, 200, 500]
    for vel in t_velocities:
        tcp_server.send_response(f"Testing position {vel}")
        
        positions = [client.hand.get_position()]
        currents = [client.hand.get_current()]
        velocities = [client.hand.get_velocity()]
        target_positions = [client.hand.get_tar_position()]
        timestamps = [time.time()]

        start_time = time.time()
        while get_average_finger_positions(positions[-1]) < 90:
            target_positions.append(client.hand.get_tar_position())
            positions.append(client.hand.get_position())
            # Update target
            target_pos = vel * SLEEP_TIME
            new_pos = [i + target_pos for i in target_positions[-1]]
            new_pos[-1], new_pos[-2] = (-START_POS, START_POS)
            client.set_position(positions=new_pos, reply_mode=2)
            currents.append(client.hand.get_current())
            velocities.append(client.hand.get_velocity())
            timestamps.append(time.time())
            time.sleep(SLEEP_TIME)

        while get_average_finger_positions(positions[-1]) > 10:
            target_positions.append(client.hand.get_tar_position())
            positions.append(client.hand.get_position())
            # Update target
            target_pos = vel * SLEEP_TIME
            new_pos = [i - target_pos for i in target_positions[-1]]
            new_pos[-1], new_pos[-2] = (-START_POS, START_POS)
            client.set_position(positions=new_pos, reply_mode=2)
            currents.append(client.hand.get_current())
            velocities.append(client.hand.get_velocity())
            timestamps.append(time.time())
            time.sleep(SLEEP_TIME)
        time_string = f"Total Time: {time.time() - start_time:.2f} Target Time: {(END_POS - START_POS) * 2 / vel:.2f}"

        # Send data over TCP
        test_data = {
            "test_type": "position_validation",
            "target_velocity": vel,
            "positions": positions,
            "currents": currents,
            "velocities": velocities,
            "target_positions": target_positions,
            "timestamps": timestamps,
            "time_string": time_string
        }
        tcp_server.send_data(test_data)

        fig, axs = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
        titles = (
            "Position",
            "Current",
            "Velocity",
            "Target - Actual Position",
        )
        data = (
            positions,
            currents,
            velocities,
            [
                list(np.array(target_positions[i]) - np.array(positions[i]))
                for i in range(len(positions))
            ],
        )

        for i in range(4):
            values = list(zip(*data[i]))
            for j in range(4):
                axs[i].plot(
                    timestamps, values[j], color=colors[j], label=fingers[j]
                )
            axs[i].set_ylabel(titles[i])
            if i == 0:
                axs[i].legend()
            axs[i].grid(True)

        axs[3].set_xlabel("Timestamp")
        plt.suptitle(
            "POSITION MODE \n"
            + f"Positions, Currents, Velocities Vs. Time : Target Velocity = {vel} dps"
            + "\n"
            + time_string
        )
        plt.tight_layout()
        plt.show()

        # Thumb flexor test
        tcp_server.send_response("Testing thumb flexor position")
        client.set_position(
            [START_POS, START_POS, START_POS, START_POS, START_POS, -START_POS]
        )
        time.sleep(1)
        positions = [client.hand.get_position()]
        currents = [client.hand.get_current()]
        velocities = [client.hand.get_velocity()]
        target_positions = [client.hand.get_tar_position()]
        timestamps = [time.time()]

        start_time = time.time()
        while positions[-1][4] < 90:
            target_positions.append(client.hand.get_tar_position())
            positions.append(client.hand.get_position())
            # Update target
            target_pos = vel * SLEEP_TIME
            new_pos = target_positions[-1].copy()
            new_pos[4] += target_pos
            new_pos[5] -= target_pos
            client.set_position(positions=new_pos, reply_mode=2)
            currents.append(client.hand.get_current())
            velocities.append(client.hand.get_velocity())
            timestamps.append(time.time())
            time.sleep(SLEEP_TIME)

        while positions[-1][4] > 10:
            target_positions.append(client.hand.get_tar_position())
            positions.append(client.hand.get_position())
            # Update target
            target_pos = vel * SLEEP_TIME
            new_pos = target_positions[-1].copy()
            new_pos[4] -= target_pos
            new_pos[5] += target_pos
            client.set_position(positions=new_pos, reply_mode=2)
            currents.append(client.hand.get_current())
            velocities.append(client.hand.get_velocity())
            timestamps.append(time.time())
            time.sleep(SLEEP_TIME)
        time_string = f"Total Time: {time.time() - start_time:.2f} Target Time: {(END_POS - START_POS) * 2 / vel:.2f}"

        # Send thumb data over TCP
        thumb_data = {
            "test_type": "position_validation_thumb",
            "target_velocity": vel,
            "positions": positions,
            "currents": currents,
            "velocities": velocities,
            "target_positions": target_positions,
            "timestamps": timestamps,
            "time_string": time_string
        }
        tcp_server.send_data(thumb_data)

        fig, axs = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
        titles = (
            "Position",
            "Current",
            "Velocity",
            "Target - Actual Position",
        )
        data = (
            positions,
            currents,
            velocities,
            [
                list(np.array(target_positions[i]) - np.array(positions[i]))
                for i in range(len(positions))
            ],
        )

        for i in range(4):
            values = list(zip(*data[i]))
            axs[i].plot(timestamps, values[4], label="flexor")
            axs[i].plot(
                timestamps, [-1 * i for i in values[5]], label="rotator"
            )
            axs[i].set_ylabel(titles[i])
            if i == 0:
                axs[i].legend()
            axs[i].grid(True)

        axs[2].set_xlabel("Timestamp")
        plt.suptitle(
            "POSITION MODE \n"
            f"Positions, Currents, Velocities Vs. Time : Target Velocity = {vel} dps"
            + "\n"
            + time_string
        )
        plt.tight_layout()
        plt.show()
    client.close()
    tcp_server.send_response("Position validation complete")
    return "complete"


def validate_torque():
    """Validate that torque/current mode is uniformly moving hands, feedback is correct
    and current draw among 4 fingers is roughly uniform.
    """
    tcp_server.send_response("Starting torque validation test")
    command = tcp_server.wait_for_command(["start", "skip", "quit"])
    
    if command == "skip":
        tcp_server.send_response("Skipping torque validation")
        return "skipped"
    elif command == "quit":
        return "quit"
        
    print("VALIDATING TORQUE")
    client = AHSerialClient()
    client.set_torque(currents=[0, 0, 0, 0, 0, 0], reply_mode=0)
    time.sleep(1)
    colors = ["r", "g", "b", "m"]
    fingers = ("index", "middle", "ring", "pinky")

    t_torques = [0.3, 0.5, 0.7]
    for tor in t_torques:
        tcp_server.send_response(f"Testing torque {tor}")
        
        positions = [client.hand.get_position()]
        currents = [client.hand.get_current()]
        target_torque = [client.hand.get_tar_current()]
        timestamps = [time.time()]
        if tor == t_torques[0]:
            tcp_server.send_response("RESIST AGAINST FINGER MOVEMENT TO ENSURE CURRENT TARGETS ARE MET")
            tcp_server.wait_for_command(["ready"])
            time.sleep(2)

        while get_average_finger_positions(positions[-1]) < 90:
            target_torque.append(client.hand.get_tar_current())
            positions.append(client.hand.get_position())
            client.set_torque(
                currents=[tor, tor, tor, tor, 0, 0], reply_mode=0
            )
            currents.append(client.hand.get_current())
            timestamps.append(time.time())
            time.sleep(SLEEP_TIME)

        while get_average_finger_positions(positions[-1]) > 10:
            target_torque.append(client.hand.get_tar_current())
            positions.append(client.hand.get_position())
            client.set_torque(
                currents=[-tor, -tor, -tor, -tor, 0, 0], reply_mode=0
            )
            currents.append(client.hand.get_current())
            timestamps.append(time.time())
            time.sleep(SLEEP_TIME)

        # Send data over TCP
        test_data = {
            "test_type": "torque_validation",
            "target_torque": tor,
            "positions": positions,
            "currents": currents,
            "target_torque": target_torque,
            "timestamps": timestamps
        }
        tcp_server.send_data(test_data)

        fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
        titles = ("Position", "Current", "Target - Actual Current")
        data = (
            positions,
            currents,
            [
                list(np.array(target_torque[i]) - np.array(currents[i]))
                for i in range(len(positions))
            ],
        )

        for i in range(3):
            values = list(zip(*data[i]))
            for j in range(4):
                axs[i].plot(
                    timestamps, values[j], color=colors[j], label=fingers[j]
                )
            axs[i].set_ylabel(titles[i])
            if i == 0:
                axs[i].legend()
            axs[i].grid(True)

        axs[2].set_xlabel("Timestamp")
        plt.suptitle(
            "TORQUE MODE \n"
            + f"Positions, Currents, Vs. Time : Target Torque = {tor} amps"
            + "\n"
        )
        plt.tight_layout()
        plt.show()

        # Thumb flexor test
        tcp_server.send_response("Testing thumb flexor torque")
        positions = [client.hand.get_position()]
        currents = [client.hand.get_current()]
        target_torque = [client.hand.get_tar_current()]
        timestamps = [time.time()]
        if tor == t_torques[0]:
            tcp_server.send_response("RESIST AGAINST THUMB MOVEMENT TO ENSURE CURRENT TARGETS ARE MET")
            tcp_server.wait_for_command(["ready"])
            time.sleep(2)

        while positions[-1][4] < 90:
            target_torque.append(client.hand.get_tar_current())
            positions.append(client.hand.get_position())
            client.set_torque(
                currents=[0, 0, 0, 0, tor, -tor / 4], reply_mode=0
            )
            currents.append(client.hand.get_current())
            timestamps.append(time.time())
            time.sleep(SLEEP_TIME)

        while positions[-1][4] > 10:
            target_torque.append(client.hand.get_tar_current())
            positions.append(client.hand.get_position())
            client.set_torque(
                currents=[0, 0, 0, 0, -tor, tor / 4], reply_mode=0
            )
            currents.append(client.hand.get_current())
            timestamps.append(time.time())
            time.sleep(SLEEP_TIME)

        # Send thumb data over TCP
        thumb_data = {
            "test_type": "torque_validation_thumb",
            "target_torque": tor,
            "positions": positions,
            "currents": currents,
            "target_torque": target_torque,
            "timestamps": timestamps
        }
        tcp_server.send_data(thumb_data)

        fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
        titles = ("Position", "Current", "Target - Actual Currents")
        data = (
            positions,
            currents,
            [
                list(np.array(target_torque[i]) - np.array(currents[i]))
                for i in range(len(positions))
            ],
        )

        for i in range(3):
            values = list(zip(*data[i]))
            axs[i].plot(timestamps, values[4], label="flexor")
            axs[i].plot(
                timestamps, [-1 * i for i in values[5]], label="rotator"
            )
            axs[i].set_ylabel(titles[i])
            if i == 0:
                axs[i].legend()
            axs[i].grid(True)

        axs[2].set_xlabel("Timestamp")
        plt.suptitle(
            "POSITION MODE \n"
            f"Positions, Currents : Target Torque = {tor} amps"
        )
        plt.tight_layout()
        plt.show()
    client.close()
    tcp_server.send_response("Torque validation complete")
    return "complete"


def validate_fsr():
    tcp_server.send_response("Starting FSR validation test")
    command = tcp_server.wait_for_command(["start", "skip", "quit"])
    
    if command == "skip":
        tcp_server.send_response("Skipping FSR validation")
        return "skipped"
    elif command == "quit":
        return "quit"
        
    print("VALIDATING FSRs")
    tcp_server.send_response("IS THIS A CLINICAL HAND? [y/n]")
    choice = tcp_server.wait_for_command(["y", "n"])
    if choice == "y":
        clinical = True
    else:
        clinical = False
        
    client = AHSerialClient()
    client.set_position(30, reply_mode=0)
    time.sleep(1)
    delta_fsr = 0.5

    fingers = ["index", "middle", "ring", "pinky", "thumb"]
    positions = [
        [0, 100, 100, 100, 0, 0],
        [100, 0, 100, 100, 0, 0],
        [100, 100, 0, 100, 0, 0],
        [100, 100, 100, 0, 0, 0],
        [30, 30, 30, 30, 30, -30],
    ]
    fsr_passed = [
        [False, False, False, False, False, False],
        [False, False, False, False, False, False],
        [False, False, False, False, False, False],
        [False, False, False, False, False, False],
        [False, False, False, False, False, False],
    ]

    for i in range(len(fingers)):
        if clinical and (i in (1, 2)):
            continue

        try:
            tcp_server.send_response(f"Do not touch {fingers[i]}, press ready to continue then press on FSRs")
            tcp_server.wait_for_command(["ready"])
            client.set_position(positions[i], reply_mode=0)
            base_fsr = client.hand.get_fsr()[i * 6 : i * 6 + 6]
            goal_fsr = [j + delta_fsr for j in base_fsr]
            while False in fsr_passed[i]:
                fsr = client.hand.get_fsr()[i * 6 : i * 6 + 6]
                for f in range(len(fsr)):
                    if fsr[f] >= goal_fsr[f]:
                        if not fsr_passed[i][f]:
                            fsr_passed[i][f] = True
                            tcp_server.send_response(f"FSR {f} detected!")
                    time.sleep(0.001)
            tcp_server.send_response(f"{fingers[i]} passed")

        except KeyboardInterrupt:
            tcp_server.send_response(f"Skipping {fingers[i]}")
            tcp_server.send_response("FSRs Passed: " + str(fsr_passed[i]))

    client.close()
    tcp_server.send_response("FSR validation complete")
    return "complete"


def validate_grips():
    tcp_server.send_response("Starting grip validation test")
    command = tcp_server.wait_for_command(["start", "skip"])
    
    if command == "skip":
        tcp_server.send_response("Skipping grip validation")
        return "skipped"
    elif command == "quit":
        return "quit"
        
    print("VALIDATING GRIPS")
    client = AHSerialClient(write_thread=False)

    # Keygrip test
    tcp_server.send_response("Press ready to test keygrip")
    command = tcp_server.wait_for_command(["ready", "skip", "quit"])
    if command == "skip":
        tcp_server.send_response("Skipping keygrip test")
    elif command == "quit":
        client.close()
        return "quit"
    else:
        client.set_grip(0x08)
        client.send_command()
        time.sleep(3)
        client.set_grip(0x00)
        client.send_command()
        tcp_server.send_response("Move thumb back and forth to test. Send 'next' when done.")
        
        command = tcp_server.wait_for_command(["next", "skip", "quit"])
        if command == "quit":
            client.close()
            return "quit"
        elif command == "skip":
            tcp_server.send_response("Skipping keygrip test")
        else:
            tcp_server.send_response("Keygrip test completed")

    # Hand wave test
    tcp_server.send_response("Press ready to run hand wave")
    command = tcp_server.wait_for_command(["ready", "skip", "quit"])
    if command == "skip":
        tcp_server.send_response("Skipping hand wave test")
    elif command == "quit":
        client.close()
        return "quit"
    else:
        client.set_grip(0x20)
        client.send_command()
        tcp_server.send_response("Hand wave active. Send 'next' when done.")
        
        command = tcp_server.wait_for_command(["next", "skip", "quit"])
        if command == "quit":
            client.set_grip(0x00, speed=0)
            client.send_command()
            client.close()
            return "quit"
        elif command == "skip":
            tcp_server.send_response("Skipping hand wave test")
        else:
            tcp_server.send_response("Hand wave test completed")
        
        # Clean up
        client.set_grip(0x00, speed=0)
        client.send_command()
    
    client.close()
    tcp_server.send_response("Grip validation complete")
    return "complete"


def main():
    # Start TCP server
    tcp_server.start_server()
    
    try:
        # Wait for initial command to start
        tcp_server.send_response("Validation tests ready. Send 'start' to begin, 'skip' to skip a test, or 'quit' to exit.")
        
        # Grip validation
        result = validate_grips()
        if result == "quit":
            tcp_server.send_response("Validation stopped by user")
            return
        elif result == "complete":
            command = tcp_server.wait_for_command(["next", "quit"])
            if command == "quit":
                tcp_server.send_response("Validation stopped by user")
                return
        # If skipped, continue to next test automatically
        
        # FSR validation
        result = validate_fsr()
        if result == "quit":
            tcp_server.send_response("Validation stopped by user")
            return
        elif result == "complete":
            command = tcp_server.wait_for_command(["next", "quit"])
            if command == "quit":
                tcp_server.send_response("Validation stopped by user")
                return
        # If skipped, continue to next test automatically
        
        # Velocity validation
        result = validate_velocity()
        if result == "quit":
            tcp_server.send_response("Validation stopped by user")
            return
        elif result == "complete":
            command = tcp_server.wait_for_command(["next", "quit"])
            if command == "quit":
                tcp_server.send_response("Validation stopped by user")
                return
        # If skipped, continue to next test automatically
        
        # Position validation
        result = validate_position()
        if result == "quit":
            tcp_server.send_response("Validation stopped by user")
            return
        elif result == "complete":
            command = tcp_server.wait_for_command(["next", "quit"])
            if command == "quit":
                tcp_server.send_response("Validation stopped by user")
                return
        # If skipped, continue to next test automatically
        
        # Torque validation
        result = validate_torque()
        if result == "quit":
            tcp_server.send_response("Validation stopped by user")
            return
        
        tcp_server.send_response("All validation tests complete!")
        
    except Exception as e:
        tcp_server.send_response(f"Error during validation: {e}")
    finally:
        tcp_server.close()


if __name__ == "__main__":
    main()
