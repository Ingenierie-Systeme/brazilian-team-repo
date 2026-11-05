"""ROS 2 node: keyboard teleoperation, which can take the robot at any moment (see Docs/GNC.md, section 10).

Run it in its own terminal, on the robot through ssh or on a computer of the same network:

    ros2 run gnc_control teleop

Publications, at 10 Hz while the operator has the robot:
  /teleop/active                                                          std_msgs/Bool     heading_control, depth_control and mission give way
  /cmd/tail_frequency, /cmd/tail_amplitude, /cmd/tail_bias, /cmd/pitch    std_msgs/Float64  normalized commands

If this node or its connection stops, the commands stop: the controllers do not resume a mission, the mission keeps the
tail stopped, and the watchdogs center the servos. Radio does not cross the water, so the teleop only works while the
connection reaches the robot. See teleop.py for the keys.
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64

from .shutdown import install_stop_handlers
from .teleop import TeleopState


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        p = self.declare_parameter
        self.state = TeleopState(
            frequency_hz=p('frequency_hz', 1.5).value, amplitude_deg=p('amplitude_deg', 15.0).value,
            max_frequency_hz=p('max_tail_frequency_hz', 3.0).value, max_angle_deg=p('max_tail_angle_deg', 60.0).value)
        self.pub_active = self.create_publisher(Bool, 'teleop/active', 10)
        self.pub_frequency = self.create_publisher(Float64, 'cmd/tail_frequency', 10)
        self.pub_amplitude = self.create_publisher(Float64, 'cmd/tail_amplitude', 10)
        self.pub_bias = self.create_publisher(Float64, 'cmd/tail_bias', 10)
        self.pub_mass = self.create_publisher(Float64, 'cmd/pitch', 10)
        self.create_timer(0.1, self.publish)

    def publish(self):
        if not self.state.active:
            return
        self.pub_active.publish(Bool(data=True))
        frequency, amplitude, bias, mass = self.state.commands()
        self.pub_frequency.publish(Float64(data=frequency))
        self.pub_amplitude.publish(Float64(data=amplitude))
        self.pub_bias.publish(Float64(data=bias))
        self.pub_mass.publish(Float64(data=mass))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_stop_handlers()
    node = TeleopNode()
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())                                    # keys without Enter
    print('Space swim, A/D bias, W/S moving mass, C center, X stop, R release, Q quit')
    print(node.state.describe())
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'q':
                    break
                if node.state.key(key):
                    print(node.state.describe())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
