import time
import threading
from math import pi, sin

from ah_wrapper import AHSerialClient
from ah_plotting.plots import CombinedRealTimePlot

RUNNING = True


def hand_wave_thread(hand_client):
    alternate = True
    pos = [30, 30, 30, 30, 30, -30]
    while RUNNING:
        current_time = time.time()
        for i in range(0, len(pos)):
            ft = current_time * 3 + i * (2 * pi) / 12
            pos[i] = (0.5 * sin(ft) + 0.5) * 45 + 15
        pos[5] = -pos[5]
        if alternate:
            hand_client.set_position(positions=pos, reply_mode=0)
            alternate = False
        else:
            hand_client.set_position(positions=pos, reply_mode=1)
            alternate = True

        hand_client.send_command()
        time.sleep(1 / hand_client.rate_hz)


def main():
    client = AHSerialClient(write_thread=False)
    write_thread = threading.Thread(target=hand_wave_thread, args=(client,))
    write_thread.start()
    plotter = CombinedRealTimePlot(client.hand)
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
