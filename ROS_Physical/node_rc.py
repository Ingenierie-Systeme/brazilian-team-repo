#!/usr/bin/env python3

import os
import termios
import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from navigator.config import *


def reset_uart(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.BRKINT | termios.ICRNL |
                      termios.INPCK | termios.ISTRIP |
                      termios.IXON)
        attrs[1] &= ~termios.OPOST
        attrs[2] |= termios.CS8
        attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
        attrs[2] |= termios.PARENB      # SBUS
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

    x = max(min_pwm, min(max_pwm, x))

    return (x - min_pwm) / (max_pwm - min_pwm) * 2.0 - 1.0


class RcNode(Node):

    def __init__(self):

        super().__init__("rc_receiver")

        reset_uart(RX_PORT)

        self.ser = serial.Serial(
            port=RX_PORT,
            baudrate=100000,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
            timeout=0.0,
        )

        self.buffer = bytearray()

        self.publisher = self.create_publisher(
            Float32MultiArray,
            "/input/rc",
            10,
        )

        self.timer = self.create_timer(
            0.01,          # 100 Hz
            self.read_sbus,
        )

        self.get_logger().info("SBUS receiver started.")

    def read_sbus(self):

        try:

            self.buffer += self.ser.read(64)

            while len(self.buffer) >= 25:

                if self.buffer[0] != 0x0F:
                    self.buffer.pop(0)
                    continue

                frame = bytes(self.buffer[:25])
                del self.buffer[:25]

                channels = decode_sbus(frame)
                channels = [
                    pwm_to_normalized(c)
                    for c in channels
                ]

                msg = Float32MultiArray()
                msg.data = channels

                self.publisher.publish(msg)

        except Exception as e:

            self.get_logger().warn(str(e))


def main():

    rclpy.init()

    node = RcNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()