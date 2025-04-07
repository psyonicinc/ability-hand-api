import time
import random
import sys

from ah_wrapper.ah_serial_client import AHSerialClient
from ah_wrapper.ah_api import create_misc_msg


def main():
    if len(sys.argv) != 2 or (sys.argv[1].lower() not in ('true', 'false')):
        print("Usage:\npython3 disable_ble.py <true|false>")
        print("\nI.e. python3 disable_ble.py true will disable the ble radio")
        exit(1)

    sc = AHSerialClient(
        auto_start_threads=False, write_thread=False, read_thread=False)
    sc.start_time = time.time()

    if sys.argv[1].lower() == 'true':
        msg = create_misc_msg(0x08)

    elif sys.argv[1].lower() == 'false':
        msg = create_misc_msg(0x07)
    else:
        print("how?")
        exit(1)

    sc.send_command(msg)
    time.sleep(0.01)
    sc.end_time = time.time()
    sc.close()


if __name__ == "__main__":
    main()