import sys

from ah_wrapper.ah_serial_client import AHSerialClient
from ah_wrapper.ah_api import create_misc_msg

COMMANDS = {0x07: "Enable BLE", 0x08: "Disable BLE", 0x09: "Reset Hand"}


def main():
    if len(sys.argv) == 1:
        cmd = 0x09
        print(
            "ALTERNATE USES \n#############################\nDisable BLE radio"
        )
        print("python3 send_misc_msg.py 0x07\n\nEnable BLE radio:")
        print("python3 send_misc+msg.py 0x08")
        print("#############################\n")
    else:
        cmd = int(sys.argv[1], 0)

    client = AHSerialClient(read_thread=False, write_thread=False)
    if cmd in COMMANDS:
        print(f"Sending {COMMANDS[cmd]} command")
    client.send_command(create_misc_msg(cmd=cmd, addr=0x50))
    client.close()


if __name__ == "__main__":
    main()
