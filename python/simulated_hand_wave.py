import time
from math import sin, pi
import os

from sim_hand.mujoco import AHMujocoSim
from ah_wrapper.ah_serial_client import AHSerialClient

def main():
    client = AHSerialClient(simulated=True, write_thread=False)
    sim = AHMujocoSim(hand=client.hand, scene=os.path.join('mujoco', 'unitree_z1', 'scene.xml'))
    try:
        pos = [30, 30, 30, 30, 30, -30]
        while True:
            for i in range(0, len(pos)):
                ft = time.time() * 3 + i * (2 * pi) / 12
                pos[i] = (0.5 * sin(ft) + 0.5) * 45 + 15
            pos[5] = -pos[5]
            client.set_position(positions=pos, reply_mode=2)  # Update command
            client.send_command()  # Send command
            time.sleep(1 / client.rate_hz)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()
        sim.mujuco_thread.join()

if __name__ == "__main__":
    main()