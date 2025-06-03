import time
from math import pi, sin

from ah_wrapper.ah_serial_client import AHSerialClient


def main():
    client = AHSerialClient(write_thread=False)
    """
    Since write_thread == False we will need to issue send_command after every 
    set_position command since the while loop is acting as our write thread.  One
    could alternatively generate a message using the api and use that as an argument
    for the send_command() method.  Using set_position allows you to update the 
    hands targets, which is useful for in the loop control.
    """
    try:
        pos = [30, 30, 30, 30, 30, -30]
        while True:
            current_time = time.time()
            for i in range(0, len(pos)):
                ft = current_time * 3 + i * (2 * pi) / 12
                pos[i] = (0.5 * sin(ft) + 0.5) * 45 + 15
            pos[5] = -pos[5]
            client.set_position(positions=pos, reply_mode=2)  # Update command
            client.send_command()  # Send command
            time.sleep(1 / client.rate_hz)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()


if __name__ == "__main__":
    main()
