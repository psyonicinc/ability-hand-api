#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node

from manus_ros2_msgs.msg import ManusGlove  # Make sure this message is correctly built and sourced

from plotting.plots import RealTimePlotTouch
from ah_wrapper.ah_serial_client import AHSerialClient

def map_range(value, from_min, from_max, to_min, to_max):
    # Clamp value within from_min and from_max to prevent overshooting
    value = max(min(value, from_max), from_min)
    
    # Scale value to 0-1 range
    scaled = (value - from_min) / (from_max - from_min)

    # Scale to output range
    return to_min + (scaled * (to_max - to_min))

class GloveSubscriber(Node):

    def __init__(self, client):
        super().__init__('glove_subscriber')
        self.subscription = self.create_subscription(
            ManusGlove,
            '/manus_glove_0',
            self.listener_callback,
            10  # QoS history depth
        )
        self.client = client
        self.mcps = [0,0,0,0,0,0]
        self.pips = [0,0,0,0,0,0]
        self.mcp_ranges = ((-1,80), (-2, 78), (-2,75), (-3,82), (-20,65), (-1,45))
        self.pip_ranges = ((0,90), (0,90), (0,90), (0,80), )


    def listener_callback(self, msg: ManusGlove):

        for e in msg.ergonomics:
            if e.type == "ThumbMCPStretch":
                self.mcps[-1] = map_range(e.value, 0, 45, -100, 0)
            elif e.type == "ThumbDIPStretch":
                self.mcps[-2] = map_range(e.value, -20, 65, 0 , 100)
            elif e.type == "IndexPIPStretch":
                self.mcps[0] = map_range(e.value, self.pip_ranges[0][0], self.pip_ranges[0][1], 0 , 100)
            elif e.type == "MiddlePIPStretch":
                self.mcps[1] = map_range(e.value, self.pip_ranges[1][0], self.pip_ranges[1][1], 0 , 100)
            elif e.type == "RingPIPStretch":
                self.mcps[2] = map_range(e.value, self.pip_ranges[2][0], self.pip_ranges[2][1], 0 , 100)
            elif e.type == "PinkyPIPStretch":
                self.mcps[3] = map_range(e.value, self.pip_ranges[3][0], self.pip_ranges[3][1], 0 , 100)
        
        self.client.set_position(self.mcps)

def ros_loop(client):
    rclpy.init()
    glove_subscriber = GloveSubscriber(client)
    rclpy.spin(glove_subscriber)
    glove_subscriber.destroy_node()
    rclpy.shutdown()



def main(args=None):
    client = AHSerialClient()
    ros_t = threading.Thread(target=ros_loop, args=(client,))
    ros_t.start()

    plotter = RealTimePlotTouch(client.hand)
    try:
        plotter.start()
    except KeyboardInterrupt:
        pass
    finally:
        client.close()



if __name__ == '__main__':
    main()
