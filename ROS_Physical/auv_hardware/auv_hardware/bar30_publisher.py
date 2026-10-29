#!/usr/bin/env python3
"""
ROS 2 Publisher node for Bar30 (MS5837-30BA) depth and pressure sensor on Navigator.
Publishes /depth, /pressure, and /temperature at 5 Hz.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure, Temperature

import ms5837

# Configuration
I2C_BUS = 6               # Navigator external I2C port (/dev/i2c-6)
RATE_HZ = 5.0             # 5 Hz
OSR = ms5837.OSR_8192     # Maximum oversampling (~41 ms per read)
DENSITY = 997.0           # Freshwater: 997 kg/m³ (seawater: 1029 kg/m³)
GRAVITY = 9.80665         # m/s²
SURFACE_SAMPLES = 10      # Number of readings to average for zero-depth reference


class Bar30Publisher(Node):

    def __init__(self):
        super().__init__('bar30_publisher')

        # Publishers
        self.depth_pub = self.create_publisher(Float64, 'depth', 10)
        self.pressure_pub = self.create_publisher(FluidPressure, 'pressure', 10)
        self.temp_pub = self.create_publisher(Temperature, 'temperature', 10)

        # Initialize I2C sensor
        self.sensor = ms5837.MS5837_30BA(I2C_BUS)
        if not self.sensor.init():
            self.get_logger().error(f'Bar30 not found on I2C-{I2C_BUS} (address 0x76). Check wiring.')
            raise RuntimeError(f'Could not initialize Bar30 on I2C-{I2C_BUS}')

        # Measure surface atmospheric pressure as depth=0 reference
        self.surface_pressure = self.measure_surface_pressure()
        self.get_logger().info(
            f'Bar30 ready on I2C-{I2C_BUS} (surface zero: {self.surface_pressure:.1f} mbar)'
        )

        self.timer = self.create_timer(1.0 / RATE_HZ, self.timer_callback)

    def measure_surface_pressure(self):
        """Average multiple samples to set baseline atmospheric pressure (mbar)."""
        soma = 0.0
        for _ in range(SURFACE_SAMPLES):
            self.sensor.read(OSR)
            soma += self.sensor.pressure()
        return soma / SURFACE_SAMPLES

    def timer_callback(self):
        try:
            self.sensor.read(OSR)
            pressure_mbar = self.sensor.pressure()
            temp_c = self.sensor.temperature()
            depth_m = (pressure_mbar - self.surface_pressure) * 100.0 / (DENSITY * GRAVITY)
        except OSError as err:
            self.get_logger().warn(f'I2C read error: {err}')
            return

        now = self.get_clock().now().to_msg()

        # 1. Depth (m)
        depth_msg = Float64()
        depth_msg.data = float(depth_m)
        self.depth_pub.publish(depth_msg)

        # 2. Fluid Pressure (Pa = mbar * 100)
        pressure_msg = FluidPressure()
        pressure_msg.header.stamp = now
        pressure_msg.header.frame_id = 'pressure_link'
        pressure_msg.fluid_pressure = float(pressure_mbar * 100.0)
        pressure_msg.variance = 400.0
        self.pressure_pub.publish(pressure_msg)

        # 3. Temperature (°C)
        temp_msg = Temperature()
        temp_msg.header.stamp = now
        temp_msg.header.frame_id = 'pressure_link'
        temp_msg.temperature = float(temp_c)
        temp_msg.variance = 0.04
        self.temp_pub.publish(temp_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = Bar30Publisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except RuntimeError as e:
        print(f'[ERROR] {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
