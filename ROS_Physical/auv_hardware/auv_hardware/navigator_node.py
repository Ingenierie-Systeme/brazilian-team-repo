#!/usr/bin/env python3
"""
Unified ROS 2 Hardware & Control Node for Blue Robotics Navigator.
Integrates in a single node:
  1. RC Receiver (SBUS via Navigator RC IN port):
      • Reads 8 channels at 100 Hz from UART (/dev/ttyAMA0 or /dev/serial0)
      • Publishes raw channels to /input/rc (std_msgs/Float32MultiArray)
  2. Autonomous & RC Motion Mixer:
      • Ch0 (Rudder): Offsets the tail fin angle for steering
      • Ch1 (Pitch):  Moves internal sliding mass safely within [-1.0, 0.3]
      • Ch2 (Thrust): Controls tail fin flapping oscillation (amplitude/speed)
      • Ch4 (Arm):    Safety arm/disarm switch
  3. Actuator Driver (PCA9685 PWM):
      • Channel 0: Tail fin servo (Thrust + Rudder combined)
      • Channel 1: Sliding mass servo (Pitch)
      • Also accepts external overrides on /cmd/thrust and /cmd/mass
  4. Onboard Inertial Sensing:
      • /imu/data   (sensor_msgs/Imu)           at 50 Hz
      • /imu/mag    (sensor_msgs/MagneticField) at 20 Hz
"""

import os
import sys
import math
import time
import termios
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray
from sensor_msgs.msg import Imu, MagneticField

import bluerobotics_navigator as navigator

# ----------------- Configuration & Pinout -----------------
# PWM Channels (PCA9685 on Navigator)
THRUST_CHANNEL = 0   # Canal 0: Cauda (propulsão + leme)
MASS_CHANNEL = 1     # Canal 1: Massa móvel interna (arfagem)

FREQ_HZ = 50
ZERO_US = 1500
MIN_US = 1000
MAX_US = 2000
PERIOD_US = 1_000_000.0 / FREQ_HZ  # 20,000 us at 50 Hz
STEPS = 4096.0                     # PCA9685 12-bit resolution

# Physical travel limits for sliding mass
MASS_MIN_NORM = -1.0
MASS_MAX_NORM = 0.3

# Sensor update rates (Hz)
IMU_RATE_HZ = 50.0
MAG_RATE_HZ = 20.0
RC_RATE_HZ = 100.0

# Serial port for Navigator RC IN (Hardware UART on Raspberry Pi)
DEFAULT_RC_PORTS = ['/dev/ttyAMA0', '/dev/serial0', '/dev/ttyS0']


# ----------------- Low-Level SBUS Functions -----------------
def reset_uart(path):
    """Configures raw UART attributes for SBUS (100k baud, 8E2)."""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.BRKINT | termios.ICRNL |
                      termios.INPCK | termios.ISTRIP |
                      termios.IXON)
        attrs[1] &= ~termios.OPOST
        attrs[2] |= termios.CS8
        attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
        attrs[2] |= termios.PARENB      # Even Parity for SBUS
        attrs[2] &= ~termios.PARODD
        attrs[3] &= ~(termios.ECHO | termios.ECHONL |
                      termios.ICANON | termios.IEXTEN |
                      termios.ISIG)

        attrs[6][termios.VMIN] = 1
        attrs[6][termios.VTIME] = 0

        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
    finally:
        os.close(fd)


def decode_sbus(frame):
    """Decodes a 25-byte SBUS frame into 8 channels (11-bit integers)."""
    ch = [0] * 8
    ch[0] = ((frame[1] | frame[2] << 8) & 0x07FF)
    ch[1] = ((frame[2] >> 3 | frame[3] << 5) & 0x07FF)
    ch[2] = ((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF)
    ch[3] = ((frame[5] >> 1 | frame[6] << 7) & 0x07FF)
    ch[4] = ((frame[6] >> 4 | frame[7] << 4) & 0x07FF)
    ch[5] = ((frame[7] >> 7 | frame[8] << 1 | frame[9] << 9) & 0x07FF)
    ch[6] = ((frame[9] >> 2 | frame[10] << 6) & 0x07FF)
    ch[7] = ((frame[10] >> 5 | frame[11] << 3) & 0x07FF)
    return ch


def pwm_to_normalized(x, min_pwm=174, max_pwm=1810):
    """Converts raw SBUS pulse [174, 1810] to normalized [-1.0, 1.0]."""
    x = max(min_pwm, min(max_pwm, x))
    return (x - min_pwm) / (max_pwm - min_pwm) * 2.0 - 1.0


# ----------------- Navigator Unified Node -----------------
class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator_node')

        # Parameters
        self.declare_parameter('rc_port', '/dev/ttyAMA0')
        self.declare_parameter('flap_freq_hz', 1.5)
        self.declare_parameter('max_flap_amplitude', 0.5)
        self.declare_parameter('max_rudder_angle', 0.5)

        self.rc_port_param = self.get_parameter('rc_port').value
        self.flap_freq = self.get_parameter('flap_freq_hz').value
        self.max_amp = self.get_parameter('max_flap_amplitude').value
        self.max_rudder = self.get_parameter('max_rudder_angle').value

        self.last_pwm_count = {}  # Cache last written PWM counts to minimize I2C bus traffic
        self.ch5_armed = False    # True when RC switch CH5 == 1.0 (motors active, IMU paused)

        # 1. Initialize Navigator hardware ONCE
        self.init_navigator_hardware()

        # 2. Initialize RC SBUS Receiver
        self.ser = None
        self.rc_buffer = bytearray()
        self.rc_active = False
        self.last_rc_time = 0.0
        self.t0 = time.time()
        self.init_rc_receiver()

        # 3. ROS Publishers
        self.pub_rc = self.create_publisher(Float32MultiArray, '/input/rc', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_mag = self.create_publisher(MagneticField, '/imu/mag', 10)

        # 4. External Override Subscribers (allows automated GNC or script commands)
        self.sub_thrust = self.create_subscription(
            Float64, '/cmd/thrust', self.external_thrust_callback, 10
        )
        self.sub_mass = self.create_subscription(
            Float64, '/cmd/mass', self.external_mass_callback, 10
        )

        # Offsets
        self.gyro_offset = [0.0, 0.0, 0.0]
        self.accel_offset = [0.0, 0.0, 0.0]

        # 5. Timers
        self.timer_rc = self.create_timer(1.0 / RC_RATE_HZ, self.rc_timer_callback)
        self.timer_imu = self.create_timer(1.0 / IMU_RATE_HZ, self.imu_callback)
        self.timer_mag = self.create_timer(1.0 / MAG_RATE_HZ, self.mag_callback)

        self.get_logger().info(
            f'Navigator node ready!\n'
            f'  • RC Receiver: active on {self.active_rc_port}\n'
            f'  • Actuators: Ch{THRUST_CHANNEL} (Tail), Ch{MASS_CHANNEL} (Mass, limit [{MASS_MIN_NORM}, {MASS_MAX_NORM}])\n'
            f'  • Switch CH5: Armed (=1.0) -> Motors Active & IMU paused | Disarmed (!=1.0) -> IMU Active\n'
            f'  • Telemetry: /input/rc (100 Hz), /imu/data ({IMU_RATE_HZ} Hz), /imu/mag ({MAG_RATE_HZ} Hz)'
        )

    def init_navigator_hardware(self):
        try:
            navigator.set_rgb_led_strip_size(1)
            navigator.set_navigator_version(navigator.NavigatorVersion.Version2)
            navigator.set_raspberry_pi_version(navigator.Raspberry.Pi4)
        except Exception:
            pass

        navigator.init()
        navigator.set_pwm_freq_hz(FREQ_HZ)

        # Set servos to neutral (1500 us) before enabling output
        self.move_servo(THRUST_CHANNEL, ZERO_US)
        self.move_servo(MASS_CHANNEL, ZERO_US)
        navigator.set_pwm_enable(True)

    def init_rc_receiver(self):
        self.active_rc_port = None
        ports_to_try = [self.rc_port_param] + [p for p in DEFAULT_RC_PORTS if p != self.rc_port_param]

        for port in ports_to_try:
            if not os.path.exists(port):
                continue
            try:
                reset_uart(port)
                self.ser = serial.Serial(
                    port=port,
                    baudrate=100000,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_EVEN,
                    stopbits=serial.STOPBITS_TWO,
                    timeout=0.0
                )
                self.active_rc_port = port
                self.get_logger().info(f'Successfully opened RC port: {port}')
                break
            except Exception as e:
                self.get_logger().debug(f'Could not open {port}: {e}')

        if self.ser is None:
            self.active_rc_port = 'NONE'
            self.get_logger().warn(
                f'RC serial port not accessible ({ports_to_try}). '
                'Listening for external /cmd/thrust and /cmd/mass topics.'
            )

    # ------------------ PWM Actuation ------------------

    def move_servo(self, channel, pulse_us):
        """Clamp pulse width to safe limits and command PCA9685 channel."""
        pulse_us = int(max(MIN_US, min(MAX_US, round(pulse_us))))
        count = int(round(pulse_us / PERIOD_US * STEPS))
        # Only write to I2C bus if value changed (reduces I2C congestion)
        if self.last_pwm_count.get(channel) != count:
            try:
                navigator.set_pwm_channel_value(channel, count)
                self.last_pwm_count[channel] = count
            except Exception as e:
                self.get_logger().warn_throttle(5.0, f'Error setting PWM Ch{channel}: {e}')
        return pulse_us

    def _calc_target_us(self, val):
        """Converts normalized [-1.0, 1.0] or direct microseconds [1000, 2000]."""
        if -1.0 <= val <= 1.0:
            return ZERO_US + val * (MAX_US - ZERO_US)
        return val

    # ------------------ RC Receiver & Mixer ------------------

    def rc_timer_callback(self):
        if self.ser is None:
            return

        try:
            raw_bytes = self.ser.read(64)
            if raw_bytes:
                self.rc_buffer += raw_bytes

            while len(self.rc_buffer) >= 25:
                # Synchronize to SBUS start byte 0x0F
                if self.rc_buffer[0] != 0x0F:
                    self.rc_buffer.pop(0)
                    continue

                frame = bytes(self.rc_buffer[:25])
                del self.rc_buffer[:25]

                # Decode & normalize to [-1.0, 1.0]
                channels_raw = decode_sbus(frame)
                channels = [pwm_to_normalized(c) for c in channels_raw]

                # Publish raw channels for telemetry / debugging
                msg = Float32MultiArray()
                msg.data = channels
                self.pub_rc.publish(msg)

                self.rc_active = True
                self.last_rc_time = time.time()

                # Execute Control Mixer
                self.process_rc_control(channels)

        except Exception as e:
            self.get_logger().warn_throttle(5.0, f'RC read error: {e}')

        # Failsafe: if RC signal drops for > 1.0s, return actuators to neutral
        if self.rc_active and (time.time() - self.last_rc_time > 1.0):
            self.rc_active = False
            self.ch5_armed = False
            self.get_logger().warn('RC signal lost! Returning motors to neutral.')
            self.move_servo(THRUST_CHANNEL, ZERO_US)
            self.move_servo(MASS_CHANNEL, ZERO_US)

    def process_rc_control(self, channels):
        """
        Mixer mapping:
          ch[0]: Rudder (horizontal stick) -> Tail bias offset
          ch[1]: Pitch  (vertical stick)   -> Internal sliding mass [-1.0, 0.3]
          ch[2]: Thrust (throttle stick)   -> Tail flapping amplitude
          ch[4]: Switch CH5                -> Armed when positive/1.0
        """
        if len(channels) < 3:
            return

        rudder_stick = channels[0]
        pitch_stick  = channels[1]
        throttle     = channels[2]

        # Check Switch CH5 (Armed when ch5 == 1.0 / > 0.8)
        # Checks index 4 (standard Channel 5) and index 5 (if 0-based)
        ch5_val = channels[4] if len(channels) > 4 else -1.0
        if ch5_val < 0.8 and len(channels) > 5 and channels[5] > 0.8:
            ch5_val = channels[5]

        was_armed = self.ch5_armed
        self.ch5_armed = (ch5_val > 0.8)

        # Mode Transition:
        # If CH5 is NOT armed (ch5 != 1.0), the radio yields control to ROS topics (/cmd/thrust, /cmd/mass)
        if not self.ch5_armed:
            if was_armed:
                # When pilot switches off manual RC, safely return actuators to neutral once
                self.move_servo(THRUST_CHANNEL, ZERO_US)
                self.move_servo(MASS_CHANNEL, ZERO_US)
                self.get_logger().info('Manual RC disarmed (CH5 != 1.0). Motors now controlled by /cmd/thrust and /cmd/mass (Autonomous mode).')
            return

        # 1. Mass Control with physical travel limits [-1.0, 0.3]
        clamped_mass = max(MASS_MIN_NORM, min(MASS_MAX_NORM, pitch_stick))
        target_mass_us = self._calc_target_us(clamped_mass)
        self.move_servo(MASS_CHANNEL, target_mass_us)

        # 2. Tail Control (Thrust Flapping + Rudder Offset)
        t = time.time() - self.t0
        rudder_bias = max(-self.max_rudder, min(self.max_rudder, rudder_stick * self.max_rudder))

        # If throttle stick is pushed forward (above -0.8):
        if throttle > -0.8:
            throttle_norm = (throttle + 1.0) / 2.0  # Scales from 0.0 to 1.0
            amplitude = self.max_amp * throttle_norm
            flapping = amplitude * math.sin(2.0 * math.pi * self.flap_freq * t)
            tail_angle = rudder_bias + flapping
        else:
            tail_angle = rudder_bias

        tail_angle = max(-1.0, min(1.0, tail_angle))
        target_tail_us = self._calc_target_us(tail_angle)
        self.move_servo(THRUST_CHANNEL, target_tail_us)

    # ------------------ Autonomous / External Overrides ------------------

    def external_thrust_callback(self, msg):
        # Only accept external commands if manual RC pilot override is NOT active (CH5 != 1.0)
        if self.ch5_armed:
            return
        target_us = self._calc_target_us(msg.data)
        self.move_servo(THRUST_CHANNEL, target_us)

    def external_mass_callback(self, msg):
        # Only accept external commands if manual RC pilot override is NOT active (CH5 != 1.0)
        if self.ch5_armed:
            return
        clamped_val = max(MASS_MIN_NORM, min(MASS_MAX_NORM, msg.data))
        target_us = self._calc_target_us(clamped_val)
        self.move_servo(MASS_CHANNEL, target_us)

    # ------------------ IMU & Magnetometer Sensing ------------------

    def imu_callback(self):
        # Skip IMU read if Switch CH5 is armed (motors actively driven by RC)
        if self.ch5_armed:
            return

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
        # Skip Magnetometer read if Switch CH5 is armed (prevents AK09915 I2C EREMOTEIO crash)
        if self.ch5_armed:
            return

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
        if self.ser and self.ser.is_open:
            self.ser.close()


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
