"""ROS 2 node: flapping of the tail fin on the real robot (see Docs/GNC.md, section 6).

The controllers publish the same commands in the simulation and on the robot. In the simulation, RobotFishPhysics
turns them into a fin movement; on the robot, this node computes the angle the tail servo must reach at each
instant. Do not run it to command the simulator: it does not use /servo/tail.

Subscriptions:
  /cmd/tail_frequency, /cmd/tail_amplitude, /cmd/tail_bias   std_msgs/Float64, normalized (see Docs/GNC.md)
Publications:
  /servo/tail      std_msgs/Float64   servo command in [-1, 1], ±1 = full range of the servo (±500 µs)
  /gnc/fin_angle   std_msgs/Float64   fin angle asked to the servo [deg], for monitoring

Safety: without commands for `command_timeout_s`, the node keeps publishing the fin at the center. When it is
stopped (Ctrl+C or SIGTERM), it publishes the center once more. If the node crashes nothing is published, so the
servo driver needs its own watchdog.
"""
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from .tail_oscillator import TailOscillator, fin_to_servo


class TailOscillatorNode(Node):
    def __init__(self):
        super().__init__('tail_oscillator')
        # ---- parameters (defaults in config/tail_oscillator.yaml)
        p = self.declare_parameter
        self.servo_range_deg = p('servo_range_deg', 60.0).value          # fin angle at servo command 1 [deg]
        self.invert = p('invert', False).value                           # servo turning the other way
        rate = p('rate_hz', 50.0).value                                  # publication rate [Hz]
        self.oscillator = TailOscillator(
            max_frequency_hz=p('max_tail_frequency_hz', 3.0).value,      # same scale as the simulator
            max_angle_deg=p('max_tail_angle_deg', 60.0).value,           # same scale as the simulator
            command_timeout_s=p('command_timeout_s', 0.5).value)         # same watchdog as the simulator
        self.was_commanded = False                                       # to log when the state changes

        # ---- ROS interfaces
        self.create_subscription(Float64, 'cmd/tail_frequency', self.on_frequency, 10)
        self.create_subscription(Float64, 'cmd/tail_amplitude', self.on_amplitude, 10)
        self.create_subscription(Float64, 'cmd/tail_bias', self.on_bias, 10)
        self.pub_servo = self.create_publisher(Float64, 'servo/tail', 10)
        self.pub_fin = self.create_publisher(Float64, 'gnc/fin_angle', 10)
        self.create_timer(1.0 / rate, self.step)
        self.get_logger().info('tail oscillator ready, fin at the center until commands arrive')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9                 # [s]

    def on_frequency(self, msg):
        self.oscillator.set_frequency(msg.data, self.now())

    def on_amplitude(self, msg):
        self.oscillator.set_amplitude(msg.data, self.now())

    def on_bias(self, msg):
        self.oscillator.set_bias(msg.data, self.now())

    def step(self):
        """Timer: computes the fin angle of this instant and sends it to the servo."""
        now = self.now()
        self.publish_fin(self.oscillator.step(now))
        commanded = self.oscillator.is_commanded(now)
        if commanded != self.was_commanded:
            if commanded:
                self.get_logger().info('commands received, flapping')
            else:
                self.get_logger().warning('no commands: fin back to the center')
            self.was_commanded = commanded

    def publish_fin(self, fin_deg):
        self.pub_fin.publish(Float64(data=float(fin_deg)))
        self.pub_servo.publish(Float64(data=fin_to_servo(fin_deg, self.servo_range_deg, self.invert)))

    def center_fin(self):
        """Last message before stopping: leaves the fin at the center."""
        self.publish_fin(0.0)
        time.sleep(0.1)                                                  # time for the message to leave


def stop_on_signal(signum, frame):
    """Ctrl+C (SIGINT) and a stop request (SIGTERM, e.g. from a launch file or a service) end rclpy.spin()."""
    raise KeyboardInterrupt


def main(args=None):
    # Without the signal handler of rclpy, ROS stays alive after a stop request, so the node can still center the fin.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, stop_on_signal)
    signal.signal(signal.SIGTERM, stop_on_signal)
    node = TailOscillatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.center_fin()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
