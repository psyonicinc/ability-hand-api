from ah_wrapper.ah_serial_client import AHSerialClient
import time
import numpy as np
import matplotlib.pyplot as plt

SLEEP_TIME = 0.01


def get_average_finger_positions(positions: list):
    return np.mean(positions[0:4])


def validate_velocity():
    """Validate that velocity mode is uniformly moving hands, feedback is correct
    and current draw among 4 fingers is roughly uniform.
    """
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


def validate_position():
    """Validate that position mode is uniformly moving hands, feedback is correct
    and current draw among 4 fingers is roughly uniform.
    """
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


def validate_torque():
    """Validate that torque/current mode is uniformly moving hands, feedback is correct
    and current draw among 4 fingers is roughly uniform.
    """
    print("VALIDATING TORQUE")
    client = AHSerialClient()
    client.set_torque(currents=[0, 0, 0, 0, 0, 0], reply_mode=0)
    time.sleep(1)
    colors = ["r", "g", "b", "m"]
    fingers = ("index", "middle", "ring", "pinky")

    t_torques = [0.3, 0.5, 0.7]
    for tor in t_torques:
        positions = [client.hand.get_position()]
        currents = [client.hand.get_current()]
        target_torque = [client.hand.get_tar_current()]
        timestamps = [time.time()]
        if tor == t_torques[0]:
            input(
                "RESIST AGAINST FINGER MOVEMENT TO ENSURE CURRENT TARGETS ARE MET PRESS ENTER TO CONTINUE"
            )
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
        positions = [client.hand.get_position()]
        currents = [client.hand.get_current()]
        target_torque = [client.hand.get_tar_current()]
        timestamps = [time.time()]
        if tor == t_torques[0]:
            input(
                "RESIST AGAINST THUMB MOVEMENT TO ENSURE CURRENT TARGETS ARE MET PRESS ENTER TO CONTINUE"
            )
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


def validate_fsr():
    print("VALIDATING FSRs")
    choice = input("IS THIS A CLINICAL HAND? [y/N]")
    if choice.lower() in ("y", "yes"):
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
            input(
                f"Do not touch {fingers[i]}, press enter to continue then press on FSRs press ctrl+c at any point to skip"
            )
            client.set_position(positions[i], reply_mode=0)
            base_fsr = client.hand.get_fsr()[i * 6 : i * 6 + 6]
            goal_fsr = [j + delta_fsr for j in base_fsr]
            while False in fsr_passed[i]:
                fsr = client.hand.get_fsr()[i * 6 : i * 6 + 6]
                for f in range(len(fsr)):
                    if fsr[f] >= goal_fsr[f]:
                        if not fsr_passed[i][f]:
                            fsr_passed[i][f] = True
                            print(f"FSR {f} detected!")
                    time.sleep(0.001)
            print(f" {fingers[i]} passed")

        except KeyboardInterrupt:
            print(f"Skipping {fingers[i]}")
            print("FSRs Passed: " + str(fsr_passed[i]))

    client.close()


def validate_grips():
    print("VALIDATING GRIPS")
    client = AHSerialClient(write_thread=False)
    command_sent = False

    input("Press enter to test keygrip press ctrl-c to stop")
    try:
        while True:
            if not command_sent:
                client.set_grip(0x08)
                client.send_command()
                time.sleep(3)
                client.set_grip(0x00)
                client.send_command()
                print("Move thumb back and forth to test")
                command_sent = True
            time.sleep(0.01)
    except KeyboardInterrupt:
        client.set_grip(0x00, speed=0)
        client.send_command()
        command_sent = False

    input("Press enter to run hand wave press ctrl-c to stop")
    try:
        while True:
            if not command_sent:
                client.set_grip(0x20)
                client.send_command()
                command_sent = True
            time.sleep(0.01)
    except KeyboardInterrupt:
        client.set_grip(0x20, speed=0)
        client.send_command()
    client.close()


def main():
    validate_grips()
    validate_fsr()
    validate_velocity()
    validate_position()
    validate_torque()


if __name__ == "__main__":
    main()
