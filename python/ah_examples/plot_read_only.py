import time
import threading
from math import pi, sin

from ah_wrapper import AHSerialClient
from ah_plotting.plots import CombinedRealTimePlot
from ah_wrapper.ah_api import create_misc_msg

RUNNING = True

def read_only(hand_client, read_command=0xA0):

    """

    Args:

        read_command: The read command to use (0xA0, 0xA1, or 0xA2)

        duration: How long to read data (in seconds)

    """

    print(f"Starting read-only mode example with command 0x{read_command:02X}")


    while RUNNING:

        # Send a read-only command (A0, A1, or A2) to request data

        read_cmd = create_misc_msg(cmd=read_command, addr=hand_client.hand.addr)

        hand_client.send_command(read_cmd)


        time.sleep(1 / hand_client.rate_hz)



def main():
    client = AHSerialClient(write_thread=False)
    client.reply_mode = 1
    write_thread = threading.Thread(target=read_only, args=(client,))
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




