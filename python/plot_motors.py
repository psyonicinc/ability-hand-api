import time
import threading
from math import pi, sin

from ah_wrapper.ah_api import create_pos_msg
from ah_wrapper.ah_serial_client import AHSerialClient
from plotting.plots import RealTimePlotMotors

RUNNING = True


def hand_wave_thread(hand_client):
    try:
        pos = [30, 30, 30, 30, 30, -30]
        while RUNNING:
            for i in range(0, len(pos)):
                ft = time.time() * 3 + i * (2 * pi) / 12
                pos[i] = (0.5 * sin(ft) + 0.5) * 45 + 15
            pos[5] = -pos[5]
            hand_client.set_position(positions=pos, reply_mode=2)
            hand_client.send_command()
            time.sleep(1 / hand_client.rate_hz)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    client = AHSerialClient(write_thread=False, baud_rate=460800)
    write_thread = threading.Thread(target=hand_wave_thread, args=(client,))
    write_thread.start()
    plotter = RealTimePlotMotors(client.hand)
    try:
        plotter.start()
    except KeyboardInterrupt:
        pass
    finally:
        RUNNING = False
        client.close()
