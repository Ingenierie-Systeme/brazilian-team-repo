"""ROS 2 node: depth control of the robot fish (GNC step 4, see Docs/GNC.md, section 9).

Subscriptions:
  /state/depth, /state/depth_rate, /state/pitch   std_msgs/Float64    from the navigation
  /state/attitude     sensor_msgs/Imu             pitch rate, from the gyroscope
  /state/odom         nav_msgs/Odometry           estimated speed, to know if the robot swims
  /guidance/depth     std_msgs/Float64            depth setpoint [m]; until the first message, the node holds the
                                                  depth it measures at start
  /teleop/active      std_msgs/Bool               an operator has the robot: this node publishes nothing
Publications:
  /cmd/pitch          std_msgs/Float64            moving mass, −1 … 1, + = forward, nose down, 20 Hz
  /gnc/depth_error    std_msgs/Float64            setpoint − depth [m]
  /gnc/pitch_setpoint std_msgs/Float64            [deg], + = nose down

Safety: the setpoint is limited to [min_depth_m, max_depth_m]. If the state stops for `sensor_timeout_s`, the node
stops publishing, and the watchdog of the simulator or of the driver centers the moving mass.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64

from .depth_controller import DepthController
from .shutdown import install_stop_handlers


class DepthControlNode(Node):
    def __init__(self):
        super().__init__('depth_control')
        # ---- parameters (defaults in config/depth_control.yaml)
        p = self.declare_parameter
        self.min_depth = p('min_depth_m', 0.5).value                     # shallowest setpoint [m]
        self.max_depth = p('max_depth_m', 4.5).value                     # deepest setpoint [m]
        self.min_speed = p('min_speed_m_s', 0.1).value                   # below it, the robot does not swim [m/s]
        self.sensor_timeout = p('sensor_timeout_s', 0.5).value           # [s]
        self.rate = p('control_rate_hz', 20.0).value                     # [Hz]
        self.teleop_timeout = p('teleop_timeout_s', 0.5).value           # silence of the teleop [s]
        self.controller = DepthController(
            kp=p('kp', 30.0).value, ki=p('ki', 2.0).value, kd=p('kd', 30.0).value,
            max_pitch_deg=p('max_pitch_deg', 20.0).value,
            kp_pitch=p('kp_pitch', 0.01).value, ki_pitch=p('ki_pitch', 0.002).value, kq=p('kq', 0.005).value,
            total_mass=p('total_mass_kg', 4.713).value, moving_mass=p('moving_mass_kg', 0.713).value,
            bg=p('bg_m', 0.010).value, travel=p('moving_mass_travel_m', 0.075).value)

        # ---- state
        self.setpoint = None                                             # [m]
        self.depth = self.depth_rate = self.pitch = None
        self.pitch_rate = self.speed = 0.0                               # [deg/s], [m/s]
        self.last_state_time = None                                      # [s]
        self.teleop_time = None                                          # [s]

        # ---- ROS interfaces
        self.create_subscription(Float64, 'state/depth', self.on_depth, 10)
        self.create_subscription(Float64, 'state/depth_rate', lambda m: setattr(self, 'depth_rate', m.data), 10)
        self.create_subscription(Float64, 'state/pitch', self.on_pitch, 10)
        self.create_subscription(Imu, 'state/attitude', self.on_attitude, 10)
        self.create_subscription(Odometry, 'state/odom', self.on_odom, 10)
        self.create_subscription(Float64, 'guidance/depth', self.on_setpoint, 10)
        self.create_subscription(Bool, 'teleop/active', self.on_teleop, 10)
        self.pub_command = self.create_publisher(Float64, 'cmd/pitch', 10)
        self.pub_error = self.create_publisher(Float64, 'gnc/depth_error', 10)
        self.pub_pitch = self.create_publisher(Float64, 'gnc/pitch_setpoint', 10)
        self.create_timer(1.0 / self.rate, self.step)
        self.get_logger().info('depth control ready, waiting for the state')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9                 # [s]

    def on_depth(self, msg):
        self.depth = msg.data
        self.last_state_time = self.now()

    def on_pitch(self, msg):
        self.pitch = msg.data

    def on_attitude(self, msg):
        self.pitch_rate = math.degrees(msg.angular_velocity.y)           # FLU: + about y (left) = nose down

    def on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x

    def on_setpoint(self, msg):
        self.setpoint = max(self.min_depth, min(self.max_depth, msg.data))

    def on_teleop(self, msg):
        if msg.data:
            self.teleop_time = self.now()

    def step(self):
        """Control loop: setpoint and state → moving-mass command."""
        if self.teleop_time is not None and self.now() - self.teleop_time <= self.teleop_timeout:
            return                                                       # the operator has the robot
        if (self.depth is None or self.depth_rate is None or self.pitch is None
                or self.now() - self.last_state_time > self.sensor_timeout):
            return                                                       # no state: publish nothing (safe)
        if self.setpoint is None:
            self.setpoint = max(self.min_depth, min(self.max_depth, self.depth))
            self.get_logger().info(f'holding the initial depth {self.setpoint:.2f} m')
        command = self.controller.update(self.setpoint, self.depth, self.depth_rate, self.pitch, self.pitch_rate,
                                         self.speed >= self.min_speed, 1.0 / self.rate)
        self.pub_command.publish(Float64(data=float(command)))
        self.pub_error.publish(Float64(data=float(self.controller.error)))
        self.pub_pitch.publish(Float64(data=float(self.controller.pitch_wanted)))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_stop_handlers()
    node = DepthControlNode()
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
