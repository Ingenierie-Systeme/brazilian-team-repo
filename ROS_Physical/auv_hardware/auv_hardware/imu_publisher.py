#!/usr/bin/env python3
"""
ROS 2 Publisher node for Blue Robotics Navigator onboard IMU and Magnetometer.
Publishes:
  - /imu/data (sensor_msgs/Imu) at 50 Hz (linear acceleration & angular velocity)
  - /imu/mag (sensor_msgs/MagneticField) at 20 Hz (tri-axial magnetic field)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

import bluerobotics_navigator as navigator

# Publishing rates (Hz) matching AUV specification
IMU_RATE_HZ = 50.0  # 50 Hz for accelerometer and gyroscope
MAG_RATE_HZ = 20.0  # 20 Hz for magnetometer


class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)

        # Initialize Navigator hardware
        try:
            navigator.set_rgb_led_strip_size(1)
            navigator.set_navigator_version(navigator.NavigatorVersion.Version2)
            navigator.set_raspberry_pi_version(navigator.Raspberry.Pi4)
        except Exception:
            pass
        navigator.init()

        # Calibration offsets (can be calibrated using ~/Downloads/imu_calib scripts)
        self.gyro_offset = [0.0, 0.0, 0.0]
        self.accel_offset = [0.0, 0.0, 0.0]

        # Timers for 50 Hz IMU and 20 Hz Magnetometer
        self.imu_timer = self.create_timer(1.0 / IMU_RATE_HZ, self.imu_callback)
        self.mag_timer = self.create_timer(1.0 / MAG_RATE_HZ, self.mag_callback)

        self.get_logger().info(
            'IMU publisher ready on "/imu/data" (50 Hz) and "/imu/mag" (20 Hz)'
        )

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
        # Approximate variance (0.05 m/s² std dev -> variance = 0.0025)
        msg.linear_acceleration_covariance[0] = 0.0025
        msg.linear_acceleration_covariance[4] = 0.0025
        msg.linear_acceleration_covariance[8] = 0.0025

        # Gyroscope (rad/s)
        msg.angular_velocity.x = float(gyro.x - self.gyro_offset[0])
        msg.angular_velocity.y = float(gyro.y - self.gyro_offset[1])
        msg.angular_velocity.z = float(gyro.z - self.gyro_offset[2])
        # Approximate variance (0.01 rad/s std dev -> variance = 0.0001)
        msg.angular_velocity_covariance[0] = 0.0001
        msg.angular_velocity_covariance[4] = 0.0001
        msg.angular_velocity_covariance[8] = 0.0001

        # Orientation is not directly estimated by raw IMU hardware (REP 145 convention)
        msg.orientation_covariance[0] = -1.0

        self.imu_pub.publish(msg)

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

        # Navigator magnetometer reads in microTesla (µT).
        # ROS 2 sensor_msgs/MagneticField requires Tesla (T) -> 1 µT = 1e-6 T.
        msg.magnetic_field.x = float(mag.x * 1e-6)
        msg.magnetic_field.y = float(mag.y * 1e-6)
        msg.magnetic_field.z = float(mag.z * 1e-6)
        # Approximate variance (0.3 µT std dev -> 1e-12)
        msg.magnetic_field_covariance[0] = 1e-12
        msg.magnetic_field_covariance[4] = 1e-12
        msg.magnetic_field_covariance[8] = 1e-12

        self.mag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

