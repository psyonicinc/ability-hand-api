import time
import threading
from math import pi, sin

from ah_wrapper import AHSerialClient
from ah_plotting.plots import RealTimePlotMotors
from ah_wrapper.ah_api import create_misc_msg

RUNNING = True


def hand_wave_thread(hand_client):
    pos = [30, 30, 30, 30, 30, -30]
    while RUNNING:
        current_time = time.time()
        for i in range(0, len(pos)):
            ft = current_time * 3 + i * (2 * pi) / 12
            pos[i] = (0.5 * sin(ft) + 0.5) * 45 + 15
        pos[5] = -pos[5]
        readonly_cmd = create_misc_msg(cmd=0xA2, addr=hand_client.hand.addr)
        hand_client.send_command(readonly_cmd)
        time.sleep(1 / hand_client.rate_hz)


def main():
    client = AHSerialClient(udp=True, udp_ip='10.0.4.140', udp_port=5067, write_thread=False)
    write_thread = threading.Thread(target=hand_wave_thread, args=(client,))
    write_thread.start()
    plotter = RealTimePlotMotors(client.hand)
    try:
        plotter.start()
    except KeyboardInterrupt:
        pass
    finally:
        global RUNNING
        RUNNING = False
        client.close()


if __name__ == "__main__":
    main()
