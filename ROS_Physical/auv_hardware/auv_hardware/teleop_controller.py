#!/usr/bin/env python3
"""
AUV Teleoperation & Motion Controller.
Translates keyboard inputs (WASD + Space, matching Unity controls) into
actuator commands:
  - Space: Forward Thrust (flaps the tail fin back-and-forth)
  - A / D: Rudder Steering (offsets the tail oscillation left / right)
  - W / S: Sliding Mass (shifts internal mass forward / backward for pitch)
  - X:     Stop / Center all actuators
  - Q:     Quit

Publishes:
  - /cmd/thrust (std_msgs/Float64): Combined tail angle [-1.0, 1.0]
  - /cmd/mass   (std_msgs/Float64): Mass position [-1.0, 1.0]
"""

import sys
import select
import termios
import tty
import time
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


USAGE_BANNER = """
==============================================================
🎮  AUV KEYBOARD TELEOPERATION CONTROLLER  (Unity Controls)
==============================================================
   [Space]   : Toggle Thrust ON/OFF (tail flapping)
   [A] / [D] : Steer Left / Right (tail bias offset)
   [W] / [S] : Pitch Down (forward) / Pitch Up (backward)
   [C]       : Re-center Rudder & Mass to 0
   [X]       : Stop all (Thrust OFF, zero all)
   [Q]       : Quit
==============================================================
"""


class TeleopController(Node):

    def __init__(self):
        super().__init__('teleop_controller')

        # Publishers to the two physical motors
        self.pub_thrust = self.create_publisher(Float64, '/cmd/thrust', 10)
        self.pub_mass = self.create_publisher(Float64, '/cmd/mass', 10)

        # Control parameters
        self.declare_parameter('rate_hz', 25.0)          # 25 Hz update loop
        self.declare_parameter('flap_freq_hz', 1.5)      # 1.5 Hz flapping cycle
        self.declare_parameter('flap_amplitude', 0.5)    # 50% amplitude of oscillation
        self.declare_parameter('rudder_step', 0.1)       # Steering step per press
        self.declare_parameter('mass_step', 0.1)         # Mass step per press

        self.rate_hz = self.get_parameter('rate_hz').value
        self.flap_freq = self.get_parameter('flap_freq_hz').value
        self.flap_amplitude = self.get_parameter('flap_amplitude').value
        self.rudder_step = self.get_parameter('rudder_step').value
        self.mass_step = self.get_parameter('mass_step').value

        # Robot internal states
        self.thrust_active = False
        self.rudder_offset = 0.0  # [-0.6, 0.6]
        self.mass_position = 0.0  # [-1.0, 1.0]

        self.t0 = time.time()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

        print(USAGE_BANNER)
        self.print_status()

    def print_status(self):
        status_thrust = 'ON (Flapping)' if self.thrust_active else 'OFF'
        sys.stdout.write(
            f'\r>> [Thrust: {status_thrust:13s}] '
            f'[Rudder: {self.rudder_offset:+5.2f}] '
            f'[Mass: {self.mass_position:+5.2f}]   '
        )
        sys.stdout.flush()

    def process_key(self, key):
        """Processes a single keystroke and updates controller states."""
        key = key.lower()

        if key == ' ':
            self.thrust_active = not self.thrust_active
        elif key == 'a':
            self.rudder_offset = max(-0.6, self.rudder_offset - self.rudder_step)
        elif key == 'd':
            self.rudder_offset = min(0.6, self.rudder_offset + self.rudder_step)
        elif key == 'w':
            self.mass_position = min(1.0, self.mass_position + self.mass_step)
        elif key == 's':
            self.mass_position = max(-1.0, self.mass_position - self.mass_step)
        elif key == 'c':
            self.rudder_offset = 0.0
            self.mass_position = 0.0
        elif key == 'x':
            self.thrust_active = False
            self.rudder_offset = 0.0
            self.mass_position = 0.0

        self.print_status()

    def control_loop(self):
        """Generates the combined tail angle and sends both motor commands."""
        t = time.time() - self.t0

        # Tail angle = Rudder offset + (Flapping oscillation if thrust is active)
        if self.thrust_active:
            oscillation = self.flap_amplitude * math.sin(2.0 * math.pi * self.flap_freq * t)
            tail_angle = self.rudder_offset + oscillation
        else:
            tail_angle = self.rudder_offset

        # Clamp safely within [-1.0, 1.0]
        tail_angle = max(-1.0, min(1.0, tail_angle))

        # 1. Publish Tail motor command (/cmd/thrust)
        msg_thrust = Float64()
        msg_thrust.data = float(tail_angle)
        self.pub_thrust.publish(msg_thrust)

        # 2. Publish Internal mass command (/cmd/mass)
        msg_mass = Float64()
        msg_mass.data = float(self.mass_position)
        self.pub_mass.publish(msg_mass)

    def stop_motors(self):
        """Publishes zero to both actuators before closing."""
        msg = Float64()
        msg.data = 0.0
        self.pub_thrust.publish(msg)
        self.pub_mass.publish(msg)


def get_key(settings):
    """Non-blocking keyboard read."""
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        return sys.stdin.read(1)
    return ''


def main(args=None):
    # Save terminal settings for raw keyboard capture
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    rclpy.init(args=args)
    controller = TeleopController()

    try:
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.02)
            key = get_key(settings)
            if key:
                if key.lower() == 'q':
                    break
                controller.process_key(key)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print('\nStopping controller and resetting motors to zero...')
        controller.stop_motors()
        time.sleep(0.2)
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('Controller closed safely.')


if __name__ == '__main__':
    main()

