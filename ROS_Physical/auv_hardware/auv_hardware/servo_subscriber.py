#!/usr/bin/env python3
"""
ROS 2 Subscriber node for Blue Robotics Navigator PWM actuators.
Controls 2 motors:
  1. Tail Motor (Thrust / Rudder combined): /cmd/thrust -> PWM Channel 0
  2. Internal Sliding Mass: /cmd/mass -> PWM Channel 1
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import bluerobotics_navigator as navigator

# Channels (0 to 15 on Navigator)
THRUST_CHANNEL = 0   # Canal PWM do motor da cauda (nadadeira propulsora / leme)
MASS_CHANNEL = 1     # Canal PWM do motor da massa móvel interna (pitch)

# PWM Configuration (standard 50 Hz servo)
FREQ_HZ = 50
ZERO_US = 1500
MIN_US = 1000
MAX_US = 2000
PERIOD_US = 1_000_000.0 / FREQ_HZ  # 20,000 us at 50 Hz
STEPS = 4096.0                     # PCA9685 12-bit resolution


class ServoSubscriber(Node):

    def __init__(self):
        super().__init__('servo_subscriber')

        # Initialize Navigator hardware
        navigator.init()
        navigator.set_pwm_freq_hz(FREQ_HZ)

        # Place both servos at neutral (1500 us) before enabling outputs
        self.move_servo(THRUST_CHANNEL, ZERO_US)
        self.move_servo(MASS_CHANNEL, ZERO_US)
        navigator.set_pwm_enable(True)

        # 1. Subscriber for Tail Motor (Thrust / Rudder)
        self.sub_thrust = self.create_subscription(
            Float64,
            '/cmd/thrust',
            self.thrust_callback,
            10
        )

        # 2. Subscriber for Internal Sliding Mass
        self.sub_mass = self.create_subscription(
            Float64,
            '/cmd/mass',
            self.mass_callback,
            10
        )

        self.get_logger().info(
            f'Actuator subscriber ready!\n'
            f'  - Motor da Cauda: canal {THRUST_CHANNEL} no topico "/cmd/thrust"\n'
            f'  - Motor da Massa: canal {MASS_CHANNEL} no topico "/cmd/mass"'
        )

    def move_servo(self, channel, pulse_us):
        """Clamp pulse width to safe limits and command PCA9685 channel."""
        pulse_us = int(max(MIN_US, min(MAX_US, round(pulse_us))))
        count = int(round(pulse_us / PERIOD_US * STEPS))
        navigator.set_pwm_channel_value(channel, count)
        return pulse_us

    def _calc_target_us(self, val):
        """Converts normalized [-1.0, 1.0] or direct microseconds [1000, 2000]."""
        if -1.0 <= val <= 1.0:
            return ZERO_US + val * (MAX_US - ZERO_US)
        return val

    def thrust_callback(self, msg):
        target_us = self._calc_target_us(msg.data)
        self.move_servo(THRUST_CHANNEL, target_us)

    def mass_callback(self, msg):
        target_us = self._calc_target_us(msg.data)
        self.move_servo(MASS_CHANNEL, target_us)

    def shutdown(self):
        self.get_logger().info('Returning both servos to zero and disabling PWM...')
        self.move_servo(THRUST_CHANNEL, ZERO_US)
        self.move_servo(MASS_CHANNEL, ZERO_US)
        time.sleep(0.5)
        navigator.set_pwm_enable(False)


def main(args=None):
    rclpy.init(args=args)
    node = ServoSubscriber()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
