import time
import random
import sys
import platform

from ah_wrapper.ah_serial_client import AHSerialClient
from ah_wrapper.ah_api import create_pos_msg


def main():
    if len(sys.argv) != 3:
        if platform.system() == "Linux":
            print(
                "Usage:\nsudo python3 speed_tests.py <RATE IN HZ [1-1000]> <NUMBER OF WRITES>"
            )
            exit(1)
        else:
            print(
                "Usage python3 speed_tests.py <RATE IN HZ [0-1000]> <NUMBER OF WRITES>"
            )
    rate_hz = int(sys.argv[1])
    writes = int(sys.argv[2])

    sc = AHSerialClient(
        auto_start_threads=False, write_thread=False, rate_hz=rate_hz
    )
    pos = random.uniform(28, 32)
    msg = create_pos_msg(
        reply_mode=1, positions=[pos, pos, pos, pos, pos, -pos]
    )

    print(f"Writing {writes} messages at rate {rate_hz} hz")
    sc.start_threads()
    for i in range(writes):
        sc.send_command(msg)
        time.sleep(1 / rate_hz)
    sc.close()


if __name__ == "__main__":
    main()
