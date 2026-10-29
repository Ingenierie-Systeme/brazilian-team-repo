#!/usr/bin/env python3
"""
ROS 2 Subscriber node for Hitec HS-5496MH servo driven by Blue Robotics Navigator.
Subscribes to /cmd/rudder and commands PWM on Navigator Ch1.
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import bluerobotics_navigator as navigator
from bluerobotics_navigator import PwmChannel

# Configuration (Hitec HS-5496MH on Navigator)
CHANNEL = PwmChannel.Ch1
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
        self.move_servo(ZERO_US)
        navigator.set_pwm_enable(True)

        self.subscription = self.create_subscription(
            Float64,
            '/cmd/rudder',
            self.listener_callback,
            10
        )
        self.get_logger().info('Servo subscriber ready on "/cmd/rudder" (Navigator Ch1)')

    def move_servo(self, pulse_us):
        """Clamp pulse width to safe limits and command PCA9685."""
        pulse_us = int(max(MIN_US, min(MAX_US, round(pulse_us))))
        count = int(round(pulse_us / PERIOD_US * STEPS))
        navigator.set_pwm_channel_value(CHANNEL, count)
        return pulse_us

    def listener_callback(self, msg):
        val = msg.data
        # Supports normalized [-1.0, 1.0] (GNC) or direct microseconds [1000, 2000]
        if -1.0 <= val <= 1.0:
            target_us = ZERO_US + val * (MAX_US - ZERO_US)
        else:
            target_us = val

        self.move_servo(target_us)

    def shutdown(self):
        self.get_logger().info('Returning servo to zero and disabling PWM...')
        self.move_servo(ZERO_US)
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
