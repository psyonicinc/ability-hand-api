import time
from math import pi, sin

from ah_wrapper import AHSerialClient


def main():
    client = AHSerialClient(udp=True, udp_ip='10.0.4.151', udp_port=5067, write_thread=False, read_thread=False)
    """
    Since write_thread == False we will need to issue send_command after every
    set_position command since the while loop is acting as our write thread.  One
    could alternatively generate a message using the api and use that as an argument
    for the send_command() method.  Using set_position allows you to update the
    hands targets, which is useful for in the loop control.
    """
    try:
        pos = [30, 30, 30, 30, 30, -30]
        target = time.perf_counter()
        while True:
            current_time = time.time()
            for i in range(0, len(pos)):
                ft = current_time * 3 + i * (2 * pi) / 12
                pos[i] = (0.5 * sin(ft) + 0.5) * 45 + 15
            pos[5] = -pos[5]
            client.set_position(positions=pos, reply_mode=2)  # Update command
            client.send_command()  # Send command
            target += 0.01
            while time.perf_counter() < target:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        client.close()


if __name__ == "__main__":
    main()
