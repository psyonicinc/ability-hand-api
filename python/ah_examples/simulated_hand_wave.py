import time
from math import sin, pi
import os

from ah_wrapper.ah_serial_client import AHSerialClient
from ah_simulators.ah_mujoco import AHMujocoSim


def main():
    client = AHSerialClient(simulated=True, write_thread=False)
    l_client = AHSerialClient(simulated=True, write_thread=False)
    sim = AHMujocoSim(
        hand=client.hand,
        left_hand=l_client.hand,
        scene=os.path.join(
            "ah_simulators", "mujoco_xml", "unitree_g1", "scene.xml"
        ),
    )
    try:
        pos = [30, 30, 30, 30, 30, -30]
        while True:
            for i in range(0, len(pos)):
                ft = time.time() * 3 + i * (2 * pi) / 12
                pos[i] = (0.5 * sin(ft) + 0.5) * 45 + 15
            pos[5] = -pos[5]
            client.set_position(positions=pos, reply_mode=2)  # Update command
            l_client.set_position(
                positions=pos, reply_mode=2
            )  # Update command
            client.send_command()
            time.sleep(1 / client.rate_hz)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()
        sim.mujuco_thread.join()


if __name__ == "__main__":
    main()
