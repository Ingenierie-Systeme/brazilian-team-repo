"""ROS 2 node: heading control of the robot fish (GNC step 2, see Docs/GNC.md).

The robot swims at a constant flapping (frequency and amplitude, parameters) and the node steers it with the
bias of the flapping, so that its heading follows the setpoint.

Subscriptions:
  /state/heading     std_msgs/Float64   heading estimated by the navigation [deg, clockwise from true north]
  /guidance/heading  std_msgs/Float64   heading setpoint [deg, clockwise from true north]; until the first
                                        message, the node holds the heading it measures at start
  /guidance/swim     std_msgs/Bool      False: no flapping. Without messages, the robot swims. Once a mission has
                                        sent it, its silence for `swim_timeout_s` also stops the flapping
  /teleop/active     std_msgs/Bool      an operator has the robot: this node publishes nothing
Publications:
  /cmd/tail_frequency, /cmd/tail_amplitude, /cmd/tail_bias   std_msgs/Float64, normalized (see Docs/GNC.md)
  /gnc/heading        std_msgs/Float64   measured heading, averaged over one flapping period [deg]
  /gnc/heading_error  std_msgs/Float64   setpoint − heading [deg], + = target to the right
  /gnc/tail_bias      std_msgs/Float64   commanded fin bias [deg]

Safety: if the estimated heading stops (no message for `sensor_timeout` s), the node stops publishing; the
watchdog of the simulator or of tail_oscillator then stops the flapping after 0.5 s.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64

from .heading import CircularMovingAverage, wrap_360
from .heading_controller import HeadingPI
from .shutdown import install_stop_handlers


class HeadingControlNode(Node):
    def __init__(self):
        super().__init__('heading_control')
        # ---- parameters (defaults in config/heading_control.yaml)
        p = self.declare_parameter
        self.cruise_frequency = p('cruise_frequency_hz', 1.5).value      # flapping frequency [Hz]
        self.cruise_amplitude = p('cruise_amplitude_deg', 30.0).value    # flapping amplitude [deg]
        self.max_tail_frequency = p('max_tail_frequency_hz', 3.0).value  # = maxTailFrequency in Unity
        self.max_tail_angle = p('max_tail_angle_deg', 60.0).value        # = maxTailAngle in Unity
        self.rate = p('control_rate_hz', 20.0).value                     # control loop rate [Hz]
        self.sensor_timeout = p('sensor_timeout_s', 0.5).value           # [s]
        self.swim_timeout = p('swim_timeout_s', 2.0).value               # silence of a mission [s]
        self.teleop_timeout = p('teleop_timeout_s', 0.5).value           # silence of the teleop [s]
        self.controller = HeadingPI(
            kp=p('kp', 1.5).value,                                       # [deg bias / deg error]
            ki=p('ki', 0.02).value,                                      # [deg bias / (deg error · s)]
            max_bias_deg=p('max_bias_deg', 30.0).value)                  # [deg]
        window = p('filter_window_s', 0.0).value                         # 0 = one flapping period
        self.filter = CircularMovingAverage(window if window > 0.0 else 1.0 / self.cruise_frequency)

        # ---- state
        self.setpoint = None                                             # [deg]
        self.swim = True                                                 # latest /guidance/swim
        self.swim_time = None                                            # time of the latest one [s]
        self.teleop_time = None                                          # [s]
        self.last_heading_time = None                                    # [s]

        # ---- ROS interfaces
        self.create_subscription(Float64, 'state/heading', self.on_heading, 10)
        self.create_subscription(Float64, 'guidance/heading', self.on_setpoint, 10)
        self.create_subscription(Bool, 'guidance/swim', self.on_swim, 10)
        self.create_subscription(Bool, 'teleop/active', self.on_teleop, 10)
        self.pub_frequency = self.create_publisher(Float64, 'cmd/tail_frequency', 10)
        self.pub_amplitude = self.create_publisher(Float64, 'cmd/tail_amplitude', 10)
        self.pub_bias = self.create_publisher(Float64, 'cmd/tail_bias', 10)
        self.pub_heading = self.create_publisher(Float64, 'gnc/heading', 10)
        self.pub_error = self.create_publisher(Float64, 'gnc/heading_error', 10)
        self.pub_bias_deg = self.create_publisher(Float64, 'gnc/tail_bias', 10)
        self.create_timer(1.0 / self.rate, self.step)
        self.get_logger().info('heading control ready, waiting for the estimated heading')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9                 # [s]

    def on_heading(self, msg):
        t = self.now()
        self.filter.add(t, msg.data)
        self.last_heading_time = t

    def on_setpoint(self, msg):
        self.setpoint = wrap_360(msg.data)

    def on_swim(self, msg):
        self.swim, self.swim_time = msg.data, self.now()

    def on_teleop(self, msg):
        if msg.data:
            self.teleop_time = self.now()

    def stop_flapping(self):
        """No flapping, fin centered; the integral starts again at the next swim."""
        self.controller.reset()
        self.publish(self.pub_frequency, 0.0)
        self.publish(self.pub_amplitude, 0.0)
        self.publish(self.pub_bias, 0.0)

    def step(self):
        """Control loop: averaged heading → PI → fin bias, published with the flapping commands."""
        now = self.now()
        if self.teleop_time is not None and now - self.teleop_time <= self.teleop_timeout:
            return                                                       # the operator has the robot
        if not self.swim or (self.swim_time is not None and now - self.swim_time > self.swim_timeout):
            self.stop_flapping()
            return
        heading = self.filter.value()
        if heading is None or self.now() - self.last_heading_time > self.sensor_timeout:
            return                                                       # no estimate: publish nothing (safe)
        if self.setpoint is None:                                        # no setpoint yet: hold the current heading
            self.setpoint = heading
            self.get_logger().info(f'holding the initial heading {heading:.1f} deg')
        bias = self.controller.update(self.setpoint, heading, 1.0 / self.rate)
        self.publish(self.pub_frequency, self.cruise_frequency / self.max_tail_frequency)
        self.publish(self.pub_amplitude, self.cruise_amplitude / self.max_tail_angle)
        self.publish(self.pub_bias, bias / self.max_tail_angle)
        self.publish(self.pub_heading, heading)
        self.publish(self.pub_error, self.controller.error)
        self.publish(self.pub_bias_deg, bias)

    @staticmethod
    def publish(publisher, value):
        publisher.publish(Float64(data=float(value)))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_stop_handlers()
    node = HeadingControlNode()
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
