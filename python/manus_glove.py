#!/usr/bin/env python3
import threading
import math

from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

from manus_ros2_msgs.msg import ManusGlove  # Make sure this message is correctly built and sourced

from plotting.plots import RealTimePlotTouch, RealTimePlotMotors, CombinedRealTimePlot
from ah_wrapper.ah_serial_client import AHSerialClient
from ah_simulators.ah_mujoco import AHMujocoSim

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
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            ManusGlove,
            '/manus_glove_0',
            self.listener_callback,
            10  # QoS history depth
        )
        self.client = client
        sim = AHMujocoSim(hand=self.client.hand, scene='ah_simulators/mujoco_xml/unitree_z1/scene.xml')
        self.mcps = [0,0,0,0,0,0]
        self.pips = [0,0,0,0,0,0]
        self.mcp_ranges = ((-1,80), (-2, 78), (-2,75), (-3,82), (-20,65), (-1,45))
        self.pip_ranges = ((0,90), (0,90), (0,90), (0,80), )

        self.alternate = True



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

        if self.alternate:
            self.client.set_position(self.mcps, reply_mode=0)
            self.alternate = False
        else:
            self.client.set_position(self.mcps, reply_mode=1)
            self.alternate = True


        ori =  R.from_quat([msg.raw_sensor_orientation.x,
                           msg.raw_sensor_orientation.y,
                           msg.raw_sensor_orientation.z,
                           msg.raw_sensor_orientation.w])

        y_rot = R.from_euler('x', 90, degrees=True)

        rotated = (y_rot * ori).as_quat()

        # Publish orientation of glove
        t = TransformStamped()

        # Fill in header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'glove'

        # Use fixed translation
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0

        # Use the received quaternion
        t.transform.rotation.x = rotated[0]
        t.transform.rotation.y = rotated[1]
        t.transform.rotation.z = rotated[2]
        t.transform.rotation.w = rotated[3]

        # Send the transform
        self.tf_broadcaster.sendTransform(t)


def ros_loop(client):
    rclpy.init()
    glove_subscriber = GloveSubscriber(client)
    rclpy.spin(glove_subscriber)
    glove_subscriber.destroy_node()
    rclpy.shutdown()


def main(args=None):
    client = AHSerialClient(simulated=True)
    ros_t = threading.Thread(target=ros_loop, args=(client,))
    ros_t.start()

    plotter = CombinedRealTimePlot(client.hand)
    try:
        plotter.start()
    except KeyboardInterrupt:
        pass
    finally:
        client.close()



if __name__ == '__main__':
    main()
