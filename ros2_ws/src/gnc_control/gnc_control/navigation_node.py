"""ROS 2 node: state of the robot for the guidance and the controllers (GNC step 3, see Docs/GNC.md, section 7).

Subscriptions:
  /state/attitude     sensor_msgs/Imu    orientation estimated by the node fusion
  /depth              std_msgs/Float64   depth from the pressure sensor [m]
  /cmd/tail_frequency, /cmd/tail_amplitude   std_msgs/Float64   flapping commands, to predict the speed
  /gnss/fix           sensor_msgs/NavSatFix   optional: corrects the position when a fix is valid
Publications:
  /state/heading      std_msgs/Float64   heading [deg, clockwise from true north]
  /state/roll         std_msgs/Float64   [deg], + = right side down
  /state/pitch        std_msgs/Float64   [deg], + = nose down
  /state/depth        std_msgs/Float64   filtered depth [m]
  /state/depth_rate   std_msgs/Float64   vertical speed [m/s], + = going down
  /state/odom         nav_msgs/Odometry  position by dead reckoning (origin at the start, NWU), orientation relative
                                         to true north, estimated speed

Each topic is published only when its input arrives, so a stopped sensor stops its outputs. /state/odom needs a recent
attitude: without it, the position is neither updated nor published.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64

from .dead_reckoning import DeadReckoning, GnssCorrection
from .depth_filter import DepthFilter
from .heading import heading_from_quaternion, roll_pitch_from_quaternion, wrap_360
from .shutdown import install_stop_handlers

# Speed of the robot in the simulation [m/s], straight swim (Docs/RobotFish_Model.md, section 5.1): one row per frequency
SPEED_FREQUENCIES = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]                 # [Hz]
SPEED_AMPLITUDES = [0.0, 15.0, 30.0, 45.0]                              # [deg]
SPEED_TABLE = [0.0, 0.0, 0.0, 0.0,
               0.0, 0.08, 0.21, 0.29,
               0.0, 0.22, 0.45, 0.61,
               0.0, 0.36, 0.69, 0.90,
               0.0, 0.49, 0.90, 1.14,
               0.0, 0.61, 1.10, 1.10,
               0.0, 0.73, 0.97, 0.98]


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation')
        # ---- parameters (defaults in config/navigation.yaml)
        p = self.declare_parameter
        self.declination = p('declination_deg', -1.0).value              # magnetic north east of true north
        self.depth_filter = DepthFilter(alpha=p('depth_alpha', 0.2).value, beta=p('depth_beta', 0.02).value)
        self.depth_period = 1.0 / p('depth_rate_hz', 20.0).value         # average time between readings [s]
        self.last_depth_time = None                                      # [s]

        # ---- position: same flapping scales as the commands
        self.max_frequency = p('max_tail_frequency_hz', 3.0).value       # [Hz]
        self.max_angle = p('max_tail_angle_deg', 60.0).value             # [deg]
        self.command_timeout = p('command_timeout_s', 0.5).value         # no command: the robot stops flapping
        self.attitude_timeout = p('attitude_timeout_s', 0.5).value       # no attitude: no position
        frequencies = p('speed_frequencies_hz', SPEED_FREQUENCIES).value
        amplitudes = p('speed_amplitudes_deg', SPEED_AMPLITUDES).value
        rows = p('speed_table', SPEED_TABLE).value                       # by rows, one row per frequency
        table = [list(rows[i:i + len(amplitudes)]) for i in range(0, len(rows), len(amplitudes))]
        self.position = DeadReckoning(frequencies, amplitudes, table, speed_scale=p('speed_scale', 1.0).value,
                                      time_constant_s=p('speed_time_constant_s', 1.2).value)
        self.gnss = GnssCorrection(p('gnss_gain', 0.05).value)
        self.frequency_cmd = self.amplitude_cmd = 0.0                    # latest flapping commands, normalized
        self.last_command_time = None                                    # [s]
        self.attitude = None                                             # latest /state/attitude message
        self.attitude_time = None                                        # [s]
        self.heading = self.pitch = 0.0                                  # latest values [deg]
        self.depth = 0.0                                                 # latest filtered depth [m]
        self.last_step_time = None                                       # [s]

        # ---- ROS interfaces
        self.create_subscription(Imu, 'state/attitude', self.on_attitude, 10)
        self.create_subscription(Float64, 'depth', self.on_depth, 10)
        self.create_subscription(Float64, 'cmd/tail_frequency', self.on_frequency, 10)
        self.create_subscription(Float64, 'cmd/tail_amplitude', self.on_amplitude, 10)
        self.create_subscription(NavSatFix, 'gnss/fix', self.on_gnss, 10)
        self.pub_heading = self.create_publisher(Float64, 'state/heading', 10)
        self.pub_roll = self.create_publisher(Float64, 'state/roll', 10)
        self.pub_pitch = self.create_publisher(Float64, 'state/pitch', 10)
        self.pub_depth = self.create_publisher(Float64, 'state/depth', 10)
        self.pub_depth_rate = self.create_publisher(Float64, 'state/depth_rate', 10)
        self.pub_odom = self.create_publisher(Odometry, 'state/odom', 10)
        self.create_timer(1.0 / p('odom_rate_hz', 20.0).value, self.step_position)   # also notices a stop request
        self.get_logger().info('navigation ready, waiting for the attitude and the depth')

    def on_attitude(self, msg):
        q = msg.orientation
        roll, pitch = roll_pitch_from_quaternion(q.x, q.y, q.z, q.w)
        heading = wrap_360(heading_from_quaternion(q.x, q.y, q.z, q.w) + self.declination)
        self.publish(self.pub_heading, heading)
        self.publish(self.pub_roll, math.degrees(roll))
        self.publish(self.pub_pitch, math.degrees(pitch))
        self.heading, self.pitch = heading, math.degrees(pitch)
        self.attitude, self.attitude_time = msg, self.now()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9                 # [s]

    def on_frequency(self, msg):
        self.frequency_cmd, self.last_command_time = msg.data, self.now()

    def on_amplitude(self, msg):
        self.amplitude_cmd, self.last_command_time = msg.data, self.now()

    def on_gnss(self, msg):
        """Only valid fixes: under water the receiver publishes NO_FIX with NaN coordinates."""
        if msg.status.status >= 0 and not (math.isnan(msg.latitude) or math.isnan(msg.longitude)):
            self.gnss.apply(msg.latitude, msg.longitude, self.position)

    def step_position(self):
        """Timer: integrates the position along the latest heading and publishes it."""
        t = self.now()
        dt = 0.0 if self.last_step_time is None else t - self.last_step_time
        self.last_step_time = t
        if self.attitude is None or t - self.attitude_time > self.attitude_timeout:
            return                                                       # no attitude: no estimate (safe)
        flapping = self.last_command_time is not None and t - self.last_command_time <= self.command_timeout
        frequency = self.frequency_cmd * self.max_frequency if flapping else 0.0
        amplitude = self.amplitude_cmd * self.max_angle if flapping else 0.0
        self.position.update(frequency, amplitude, self.heading, self.pitch, dt)

        odom = Odometry()
        odom.header.stamp = self.attitude.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.position.north
        odom.pose.pose.position.y = self.position.west
        odom.pose.pose.position.z = -self.depth
        q = self.attitude.orientation                                    # rotated from magnetic to true north
        half = math.radians(-self.declination) / 2.0
        s, c = math.sin(half), math.cos(half)
        o = odom.pose.pose.orientation
        o.w, o.x, o.y, o.z = c * q.w - s * q.z, c * q.x - s * q.y, c * q.y + s * q.x, c * q.z + s * q.w
        odom.twist.twist.linear.x = self.position.speed
        odom.twist.twist.angular = self.attitude.angular_velocity
        self.pub_odom.publish(odom)

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
        self.depth = self.depth_filter.depth

    @staticmethod
    def publish(publisher, value):
        publisher.publish(Float64(data=float(value)))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_stop_handlers()
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
