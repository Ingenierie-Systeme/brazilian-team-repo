"""ROS 2 node: state of the robot for the guidance and the controllers (GNC step 3, see Docs/GNC.md, section 7).

Subscriptions:
  /state/attitude     sensor_msgs/Imu    orientation estimated by the node fusion
  /depth              std_msgs/Float64   depth from the pressure sensor [m]
Publications:
  /state/heading      std_msgs/Float64   heading [deg, clockwise from true north]
  /state/roll         std_msgs/Float64   [deg], + = right side down
  /state/pitch        std_msgs/Float64   [deg], + = nose down
  /state/depth        std_msgs/Float64   filtered depth [m]
  /state/depth_rate   std_msgs/Float64   vertical speed [m/s], + = going down

Each topic is published only when its input arrives, so a stopped sensor stops its outputs.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from .depth_filter import DepthFilter
from .heading import heading_from_quaternion, roll_pitch_from_quaternion, wrap_360


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation')
        # ---- parameters (defaults in config/navigation.yaml)
        p = self.declare_parameter
        self.declination = p('declination_deg', -1.0).value              # magnetic north east of true north
        self.depth_filter = DepthFilter(alpha=p('depth_alpha', 0.2).value, beta=p('depth_beta', 0.02).value)
        self.depth_period = 1.0 / p('depth_rate_hz', 20.0).value         # average time between readings [s]
        self.last_depth_time = None                                      # [s]

        # ---- ROS interfaces
        self.create_subscription(Imu, 'state/attitude', self.on_attitude, 10)
        self.create_subscription(Float64, 'depth', self.on_depth, 10)
        self.pub_heading = self.create_publisher(Float64, 'state/heading', 10)
        self.pub_roll = self.create_publisher(Float64, 'state/roll', 10)
        self.pub_pitch = self.create_publisher(Float64, 'state/pitch', 10)
        self.pub_depth = self.create_publisher(Float64, 'state/depth', 10)
        self.pub_depth_rate = self.create_publisher(Float64, 'state/depth_rate', 10)
        self.get_logger().info('navigation ready, waiting for the attitude and the depth')

    def on_attitude(self, msg):
        q = msg.orientation
        roll, pitch = roll_pitch_from_quaternion(q.x, q.y, q.z, q.w)
        heading = wrap_360(heading_from_quaternion(q.x, q.y, q.z, q.w) + self.declination)
        self.publish(self.pub_heading, heading)
        self.publish(self.pub_roll, math.degrees(roll))
        self.publish(self.pub_pitch, math.degrees(pitch))

    def on_depth(self, msg):
        # Readings can arrive in bursts, so the time step is the average interval, not the last one: the rate term
        # of the filter divides by it, and a very short interval would make a large peak of vertical speed.
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.last_depth_time is not None:
            self.depth_period += 0.05 * (t - self.last_depth_time - self.depth_period)
        self.last_depth_time = t
        self.depth_filter.update(msg.data, self.depth_period)
        self.publish(self.pub_depth, self.depth_filter.depth)
        self.publish(self.pub_depth_rate, self.depth_filter.rate)

    @staticmethod
    def publish(publisher, value):
        publisher.publish(Float64(data=float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
