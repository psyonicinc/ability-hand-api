import time
from math import pi, sin

from ah_wrapper.ah_serial_client import AHSerialClient
from ah_wrapper.observer import Observer
from ah_wrapper.ah_api import create_misc_msg

"""
Observer / Observable design pattern allows a method to be triggered everytime
a value in the Hand class is updated.  This is a preferred method rather than 
say using Hand.get_position() because locks are not required since the update
method is called within the hand class everytime it's data is updated.  This is 
helpful as you will always get the most recent data, and will not read previously
stored values saved in the hand class with very minimal overhead.  This is 
particular useful when you need to publish data to another thread such as a ROS 
node.
"""


class MyObserver(Observer):
    def __init__(self):
        super().__init__()

    # Must define abstract methods from Observer class
    def update_fsr(self, fsr):
        # print(f"FSR: {fsr}")
        # Abstract methods are forced to be implemented, in very least define to pass
        pass

    def update_pos(self, position):
        print(f"I saw new position feedback data {position}")

    def update_vel(self, velocity):
        # print(f"I saw new velocity feedback data {velocity}")
        pass

    def update_hot_cold(self, hot_cold):
        # if hot_cold:
            # print(f"I saw new hot cold status {hot_cold}")
        pass

    def update_cur(self, current):
        # print(f"I saw new current {current}")
        pass


def main():
    client = AHSerialClient(udp=True, udp_ip='10.0.4.151', udp_port=5067, write_thread=False)
    observer = MyObserver()
    client.hand.add_observer(observer)
    try:
        pos = [30, 30, 30, 30, 30, -30]
        while True:
            readonly_cmd = create_misc_msg(cmd=0xA0, addr=client.hand.addr)
            client.send_command(readonly_cmd)
            time.sleep(1 / client.rate_hz)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()


if __name__ == "__main__":
    main()
