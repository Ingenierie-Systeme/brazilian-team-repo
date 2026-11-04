"""ROS 2 node: depth from the Bar30 pressure sensor of the real robot (see Docs/Robot_Setup.md).

Based on node_baro.py of the course, which uses its driver lib_ms5837.py. Unlike node_baro.py, the zero of the depth is
the pressure measured when the node starts, so THE NODE MUST BE STARTED WITH THE SENSOR AT THE SURFACE.

Publications:
  /depth      std_msgs/Float64         depth [m], + = below the surface
  /pressure   sensor_msgs/FluidPressure   [Pa]
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64

from gnc_control.shutdown import install_stop_handlers

from . import lib_ms5837 as ms5837

GRAVITY = 9.80665                                       # [m/s²]


def depth_from_pressure(pressure_pa, surface_pa, density):
    """Depth [m] from the pressure [Pa], the pressure at the surface [Pa] and the water density [kg/m³]."""
    return (pressure_pa - surface_pa) / (density * GRAVITY)


class BarometerNode(Node):
    def __init__(self, sensor=None):
        super().__init__('barometer')
        # ---- parameters (defaults in config/robot.yaml)
        p = self.declare_parameter
        rate = p('rate_hz', 10.0).value                                  # [Hz]
        i2c_bus = p('i2c_bus', 6).value                                  # external I2C port of the Navigator
        self.density = p('water_density', 1000.0).value                  # [kg/m³]: fresh 997, sea 1029
        samples = p('surface_samples', 10).value                         # readings averaged for the zero

        self.sensor = sensor if sensor is not None else ms5837.MS5837_30BA(i2c_bus)
        if not self.sensor.init():
            raise RuntimeError(f'pressure sensor not found on I2C bus {i2c_bus}')

        total = 0.0
        for _ in range(samples):
            self.sensor.read()
            total += self.sensor.pressure(ms5837.UNITS_Pa)
        self.surface_pa = total / samples                                # zero of the depth [Pa]

        self.pub_depth = self.create_publisher(Float64, 'depth', 10)
        self.pub_pressure = self.create_publisher(FluidPressure, 'pressure', 10)
        self.create_timer(1.0 / rate, self.read_sensor)
        self.get_logger().info(f'pressure sensor ready, zero of the depth: {self.surface_pa:.0f} Pa')

    def read_sensor(self):
        try:
            if not self.sensor.read():
                return
            pressure_pa = self.sensor.pressure(ms5837.UNITS_Pa)
        except OSError as error:                                         # I2C error: try again at the next reading
            self.get_logger().warning(f'pressure reading failed: {error}', throttle_duration_sec=5.0)
            return
        self.pub_depth.publish(Float64(data=depth_from_pressure(pressure_pa, self.surface_pa, self.density)))
        msg = FluidPressure()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'pressure_link'
        msg.fluid_pressure = float(pressure_pa)
        self.pub_pressure.publish(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_stop_handlers()
    node = BarometerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
