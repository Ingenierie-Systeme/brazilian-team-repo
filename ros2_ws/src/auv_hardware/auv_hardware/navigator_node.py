"""ROS 2 node: IMU, magnetometer and servos of the Navigator board of the real robot (see Docs/Robot_Setup.md).

Based on node_navigator.py of the course. A single node talks to the board. It publishes the sensors with the topic
names of the simulator, so the GNC nodes are the same in both cases.

Publications:
  /imu/data   sensor_msgs/Imu             gyroscope and accelerometer, no orientation (orientation_covariance[0] = -1)
  /imu/mag    sensor_msgs/MagneticField   magnetometer [T]
Subscriptions:
  /servo/tail  std_msgs/Float64   tail servo, −1 … 1 (from tail_oscillator)
  /cmd/pitch   std_msgs/Float64   moving-mass servo, −1 … 1 (+ = forward, nose down)

The readings are not corrected here: the calibration and the sensor axes are parameters of the node fusion. The
only change is the one the course applies to the magnetometer, (x, y, z) → (−y, x, z), which also fits its calibration
tools.

Safety: at start, the servos go to the center before the outputs are enabled. Each servo returns to the center if
its commands stop for `command_timeout_s`. When the node stops, both go to the center and the outputs are disabled.
"""
import time

import bluerobotics_navigator as navigator
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64

from gnc_control.shutdown import install_stop_handlers

from .actuators import PWM_FREQUENCY_HZ, ServoChannel


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator')
        # ---- parameters (defaults in config/robot.yaml)
        p = self.declare_parameter
        imu_rate = p('imu_rate_hz', 100.0).value                         # [Hz], same value as sample_rate_hz of fusion
        mag_rate = p('mag_rate_hz', 20.0).value                          # [Hz]
        timeout = p('command_timeout_s', 0.5).value                      # [s]
        tail_channel = p('tail_channel', 0).value                        # PWM output of the tail servo
        mass_channel = p('mass_channel', 1).value                        # PWM output of the moving-mass servo
        mass_invert = p('mass_invert', False).value                      # moving mass going the other way
        mass_limit = p('mass_limit', 1.0).value                          # largest command of the moving mass

        # ---- board: the servos are centered before the outputs are enabled
        navigator.set_rgb_led_strip_size(1)
        navigator.set_navigator_version(navigator.NavigatorVersion.Version2)
        navigator.set_raspberry_pi_version(navigator.Raspberry.Pi4)
        navigator.init()
        navigator.set_pwm_freq_hz(int(PWM_FREQUENCY_HZ))
        self.tail = ServoChannel(lambda duty: navigator.set_pwm_channel_duty_cycle(tail_channel, duty), timeout)
        self.mass = ServoChannel(lambda duty: navigator.set_pwm_channel_duty_cycle(mass_channel, duty), timeout,
                                 invert=mass_invert, limit=mass_limit)
        self.tail.center()
        self.mass.center()
        navigator.set_pwm_enable(True)

        # ---- ROS interfaces
        self.pub_imu = self.create_publisher(Imu, 'imu/data', 10)
        self.pub_mag = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.create_subscription(Float64, 'servo/tail', lambda m: self.tail.command(m.data, self.now()), 10)
        self.create_subscription(Float64, 'cmd/pitch', lambda m: self.mass.command(m.data, self.now()), 10)
        self.create_timer(1.0 / imu_rate, self.read_imu)
        self.create_timer(1.0 / mag_rate, self.read_mag)
        self.create_timer(0.05, self.check_watchdogs)
        self.get_logger().info('Navigator ready, servos at the center')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9                 # [s]

    def read_imu(self):
        try:
            accel = navigator.read_accel()
            gyro = navigator.read_gyro()
        except Exception as error:                                       # a failed reading must not stop the servos
            self.get_logger().warning(f'IMU reading failed: {error}', throttle_duration_sec=5.0)
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation_covariance[0] = -1.0                             # REP 145: no orientation in this message
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
            float(accel.x), float(accel.y), float(accel.z))
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            float(gyro.x), float(gyro.y), float(gyro.z))
        self.pub_imu.publish(msg)

    def read_mag(self):
        try:
            mag = navigator.read_mag()                                   # [µT]
        except Exception as error:
            self.get_logger().warning(f'Magnetometer reading failed: {error}', throttle_duration_sec=5.0)
            return
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.magnetic_field.x = float(-mag.y) * 1e-6                      # axes of the course, in tesla
        msg.magnetic_field.y = float(mag.x) * 1e-6
        msg.magnetic_field.z = float(mag.z) * 1e-6
        self.pub_mag.publish(msg)

    def check_watchdogs(self):
        now = self.now()
        if self.tail.check_watchdog(now):
            self.get_logger().warning('no tail command: servo back to the center')
        if self.mass.check_watchdog(now):
            self.get_logger().warning('no moving-mass command: servo back to the center')

    def stop_servos(self):
        """Before stopping: both servos to the center, then the outputs off."""
        self.tail.center()
        self.mass.center()
        time.sleep(0.5)                                                  # time for the servos to get there
        navigator.set_pwm_enable(False)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_stop_handlers()
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_servos()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
