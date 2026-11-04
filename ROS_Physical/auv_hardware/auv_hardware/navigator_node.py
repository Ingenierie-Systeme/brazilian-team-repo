#!/usr/bin/env python3
"""
Unified ROS 2 Hardware Node for Blue Robotics Navigator.
Controls all onboard Navigator peripherals in a single process to avoid
hardware GPIO/EBUSY resource conflicts (e.g. exclusive LEAK pin lock):
  - Actuators:
      • /cmd/thrust (Float64) -> PWM Channel 0 (Tail motor / rudder combined)
      • /cmd/mass   (Float64) -> PWM Channel 1 (Internal sliding mass, [-1.0, 0.3])
  - Sensors:
      • /imu/data   (Imu)           at 50 Hz (accel & gyro)
      • /imu/mag    (MagneticField) at 20 Hz (tri-axial magnetometer)
"""

import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, MagneticField

import bluerobotics_navigator as navigator

# PWM Configuration (standard 50 Hz servos on PCA9685)
THRUST_CHANNEL = 0   # Canal 0: Cauda (propulsão e curva)
MASS_CHANNEL = 1     # Canal 1: Massa deslizante interna (arfagem)

FREQ_HZ = 50
ZERO_US = 1500
MIN_US = 1000
MAX_US = 2000
PERIOD_US = 1_000_000.0 / FREQ_HZ  # 20,000 us at 50 Hz
STEPS = 4096.0                     # PCA9685 12-bit resolution

# Mass physical travel limits
MASS_MIN_NORM = -1.0
MASS_MAX_NORM = 0.3

# Sensor update rates (Hz)
IMU_RATE_HZ = 50.0
MAG_RATE_HZ = 20.0


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator_node')

        # 1. Initialize Navigator hardware ONCE
        try:
            navigator.set_rgb_led_strip_size(1)
            navigator.set_navigator_version(navigator.NavigatorVersion.Version2)
            navigator.set_raspberry_pi_version(navigator.Raspberry.Pi4)
        except Exception:
            pass

        navigator.init()
        navigator.set_pwm_freq_hz(FREQ_HZ)

        # Set neutral pulse (1500 us) before enabling output
        self.move_servo(THRUST_CHANNEL, ZERO_US)
        self.move_servo(MASS_CHANNEL, ZERO_US)
        navigator.set_pwm_enable(True)

        # 2. Actuator Subscribers
        self.sub_thrust = self.create_subscription(
            Float64,
            '/cmd/thrust',
            self.thrust_callback,
            10
        )
        self.sub_mass = self.create_subscription(
            Float64,
            '/cmd/mass',
            self.mass_callback,
            10
        )

        # 3. IMU & Magnetometer Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_mag = self.create_publisher(MagneticField, '/imu/mag', 10)

        # Offsets (can be updated via calibration scripts in ~/Downloads/imu_calib)
        self.gyro_offset = [0.0, 0.0, 0.0]
        self.accel_offset = [0.0, 0.0, 0.0]

        # Timers
        self.timer_imu = self.create_timer(1.0 / IMU_RATE_HZ, self.imu_callback)
        self.timer_mag = self.create_timer(1.0 / MAG_RATE_HZ, self.mag_callback)

        self.get_logger().info(
            f'Navigator unified hardware node ready!\n'
            f'  - Actuators: /cmd/thrust (Ch{THRUST_CHANNEL}), /cmd/mass (Ch{MASS_CHANNEL}, range [{MASS_MIN_NORM}, {MASS_MAX_NORM}])\n'
            f'  - Sensors:   /imu/data ({IMU_RATE_HZ} Hz), /imu/mag ({MAG_RATE_HZ} Hz)'
        )

    # ------------------ PWM Actuation ------------------

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
        # Clamped to mechanical limit [-1.0, 0.3]
        clamped_val = max(MASS_MIN_NORM, min(MASS_MAX_NORM, msg.data))
        target_us = self._calc_target_us(clamped_val)
        self.move_servo(MASS_CHANNEL, target_us)

    # ------------------ IMU Sensing ------------------

    def imu_callback(self):
        try:
            accel = navigator.read_accel()
            gyro = navigator.read_gyro()
        except Exception as e:
            self.get_logger().warn_throttle(5.0, f'Error reading IMU: {e}')
            return

        now = self.get_clock().now().to_msg()
        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = 'imu_link'

        # Accelerometer (m/s²)
        msg.linear_acceleration.x = float(accel.x - self.accel_offset[0])
        msg.linear_acceleration.y = float(accel.y - self.accel_offset[1])
        msg.linear_acceleration.z = float(accel.z - self.accel_offset[2])
        msg.linear_acceleration_covariance[0] = 0.0025
        msg.linear_acceleration_covariance[4] = 0.0025
        msg.linear_acceleration_covariance[8] = 0.0025

        # Gyroscope (rad/s)
        msg.angular_velocity.x = float(gyro.x - self.gyro_offset[0])
        msg.angular_velocity.y = float(gyro.y - self.gyro_offset[1])
        msg.angular_velocity.z = float(gyro.z - self.gyro_offset[2])
        msg.angular_velocity_covariance[0] = 0.0001
        msg.angular_velocity_covariance[4] = 0.0001
        msg.angular_velocity_covariance[8] = 0.0001

        msg.orientation_covariance[0] = -1.0
        self.pub_imu.publish(msg)

    def mag_callback(self):
        try:
            mag = navigator.read_mag()
        except Exception as e:
            self.get_logger().warn_throttle(5.0, f'Error reading Magnetometer: {e}')
            return

        now = self.get_clock().now().to_msg()
        msg = MagneticField()
        msg.header.stamp = now
        msg.header.frame_id = 'imu_link'

        # Convert microTesla (µT) to Tesla (T)
        msg.magnetic_field.x = float(mag.x * 1e-6)
        msg.magnetic_field.y = float(mag.y * 1e-6)
        msg.magnetic_field.z = float(mag.z * 1e-6)
        msg.magnetic_field_covariance[0] = 1e-12
        msg.magnetic_field_covariance[4] = 1e-12
        msg.magnetic_field_covariance[8] = 1e-12

        self.pub_mag.publish(msg)

    def shutdown(self):
        self.get_logger().info('Shutting down Navigator hardware: returning servos to zero and disabling PWM...')
        self.move_servo(THRUST_CHANNEL, ZERO_US)
        self.move_servo(MASS_CHANNEL, ZERO_US)
        time.sleep(0.5)
        navigator.set_pwm_enable(False)


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()

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

