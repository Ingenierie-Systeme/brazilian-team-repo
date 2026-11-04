"""ROS 2 node: scripted depth setpoints and a CSV log, to evaluate the depth controller.

    ros2 run gnc_control depth_test --ros-args -p sequence:="2.5:15,3.5:25,2.5:25" -p output:=depth.csv

`sequence`: comma-separated "depth_m:duration_s" pairs. The time starts when the robot swims.
"""
import csv

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

NAN = float('nan')


class DepthTestNode(Node):
    def __init__(self):
        super().__init__('depth_test')
        self.sequence = [tuple(float(x) for x in item.split(':'))
                         for item in self.declare_parameter('sequence', '2.5:15,3.5:25').value.split(',')]
        self.output = self.declare_parameter('output', 'depth_test.csv').value
        self.values = dict(depth=NAN, rate=NAN, pitch=NAN, pitch_setpoint=NAN, command=NAN, speed=0.0)
        for topic, key in (('state/depth', 'depth'), ('state/depth_rate', 'rate'), ('state/pitch', 'pitch'),
                           ('gnc/pitch_setpoint', 'pitch_setpoint'), ('cmd/pitch', 'command')):
            self.create_subscription(Float64, topic, lambda m, k=key: self.values.__setitem__(k, m.data), 10)
        self.create_subscription(Odometry, 'state/odom', self.on_odom, 10)
        self.pub = self.create_publisher(Float64, 'guidance/depth', 10)
        self.rows = []
        self.start = None
        self.create_timer(0.05, self.step)

    def on_odom(self, msg):
        self.values['speed'] = msg.twist.twist.linear.x

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start is None:
            if self.values['speed'] < 0.05:
                self.pub.publish(Float64(data=self.sequence[0][0]))
                return                                                   # the robot does not swim yet
            self.start = now
        t = now - self.start
        elapsed = 0.0
        for depth, duration in self.sequence:
            elapsed += duration
            if t < elapsed:
                break
        else:
            with open(self.output, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'setpoint_m', 'depth_m', 'rate_m_s', 'pitch_deg', 'pitch_setpoint_deg', 'command', 'speed'])
                w.writerows(self.rows)
            self.get_logger().info(f'{len(self.rows)} rows written to {self.output}')
            raise SystemExit
        self.pub.publish(Float64(data=depth))
        v = self.values
        self.rows.append((round(t, 3), depth, v['depth'], v['rate'], v['pitch'], v['pitch_setpoint'], v['command'], v['speed']))


def main(args=None):
    rclpy.init(args=args)
    node = DepthTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
