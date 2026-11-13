"""ROS 2 node: orientation of the robot from its raw sensors (GNC step 3, see Docs/GNC.md, section 7).

Based on node_fusion.py of the course. It uses the sensor topics of the simulator and of the robot driver, and
never reads the orientation field of /imu/data, which only the simulation has.

Subscriptions:
  /imu/data          sensor_msgs/Imu             only the angular velocity and the linear acceleration
  /imu/mag           sensor_msgs/MagneticField   magnetometer [T]
Publications:
  /state/attitude    sensor_msgs/Imu             estimated orientation (body FLU → world NWU, magnetic north) and
                                                 calibrated angular velocity

Safety: nothing is published while the filter converges at start, nor if the magnetometer stops for
`sensor_timeout_s`. If the IMU stops, nothing is published either, since it drives the node. The nodes that use
the estimate then stop, and their actuators go to the safe state.

The filter advances by 1/`sample_rate_hz` at each IMU message, so this parameter must be the real rate of the IMU: at
a lower real rate, the rotations would be underestimated. The node measures the rate and warns if it differs by more
than 10 %.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, MagneticField

from .attitude_filter import AttitudeFilter
from .shutdown import install_stop_handlers


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion')
        # ---- parameters (defaults in config/navigation.yaml)
        p = self.declare_parameter
        self.sensor_timeout = p('sensor_timeout_s', 0.5).value           # [s]
        rate = p('sample_rate_hz', 50.0).value                           # rate of the IMU [Hz]
        self.dt = 1.0 / rate                                             # time step of the filter [s]
        self.filter = AttitudeFilter(
            sample_rate_hz=rate,
            gain=p('gain', 0.5).value,
            acceleration_rejection_deg=p('acceleration_rejection_deg', 10.0).value,
            magnetic_rejection_deg=p('magnetic_rejection_deg', 10.0).value,
            rejection_timeout_s=p('rejection_timeout_s', 5.0).value,
            axes=p('axes', 'PXPYPZ').value,
            gyro_scale=p('gyro_scale', 1.0).value,
            gyro_offset=p('gyro_offset', [0.0, 0.0, 0.0]).value,
            accel_offset=p('accel_offset', [0.0, 0.0, 0.0]).value,
            accel_matrix=p('accel_matrix', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]).value,
            mag_hard=p('mag_hard', [0.0, 0.0, 0.0]).value,
            mag_soft=p('mag_soft', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]).value)

        # ---- state
        self.mag_ut = None                                               # latest magnetometer reading [µT]
        self.last_mag_time = None                                        # [s]
        self.was_ready = False                                           # to log when the filter is ready
        self.rate = rate                                                 # expected IMU rate [Hz]
        self.rate_start = None                                           # start of the rate measurement [s]
        self.rate_count = 0                                              # IMU messages since rate_start

        # ---- ROS interfaces
        self.create_subscription(Imu, 'imu/data', self.on_imu, 10)
        self.create_subscription(MagneticField, 'imu/mag', self.on_mag, 10)
        self.pub = self.create_publisher(Imu, 'state/attitude', 10)
        self.create_timer(0.1, lambda: None)                             # wakes the node, so it notices a stop request
        self.get_logger().info('fusion ready, waiting for the IMU and the magnetometer')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9                 # [s]

    def on_mag(self, msg):
        f = msg.magnetic_field
        self.mag_ut = (f.x * 1e6, f.y * 1e6, f.z * 1e6)                  # the calibration values are in µT
        self.last_mag_time = self.now()

    def on_imu(self, msg):
        """Each IMU message advances the filter; the magnetometer is slower, its latest reading is reused.

        The time step is the sensor period, not the time between arrivals: messages can arrive in bursts (each
        one still covers one period of the sensor), and using their arrival times multiplied the error of the
        estimate by about 4 in the simulation (Docs/GNC.md, section 7.3).
        """
        self.check_rate()
        if self.mag_ut is None or self.now() - self.last_mag_time > self.sensor_timeout:
            return                                                       # no magnetometer: no estimate (safe)

        w, a = msg.angular_velocity, msg.linear_acceleration
        self.filter.update((w.x, w.y, w.z), (a.x, a.y, a.z), self.mag_ut, self.dt)
        if not self.filter.ready:
            return                                                       # still converging

        if not self.was_ready:
            self.was_ready = True
            self.get_logger().info('attitude estimate ready')
        out = Imu()
        out.header = msg.header
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = self.filter.quaternion
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = self.filter.gyro_rad_s
        out.linear_acceleration = msg.linear_acceleration
        self.pub.publish(out)

    def check_rate(self):
        """Counts the IMU messages over 5 s and warns if their rate is not sample_rate_hz."""
        now = self.now()
        if self.rate_start is None:
            self.rate_start = now
            return
        self.rate_count += 1
        elapsed = now - self.rate_start
        if elapsed < 5.0:
            return
        measured = self.rate_count / elapsed
        if abs(measured - self.rate) > 0.1 * self.rate:
            self.get_logger().warning(f'IMU rate {measured:.0f} Hz, but sample_rate_hz is {self.rate:.0f} Hz: '
                                      'set sample_rate_hz to the real rate')
        self.rate_start, self.rate_count = now, 0


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_stop_handlers()
    node = FusionNode()
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
