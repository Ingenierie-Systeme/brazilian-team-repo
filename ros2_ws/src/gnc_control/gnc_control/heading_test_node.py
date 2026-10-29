"""ROS 2 node: scripted heading setpoints and a CSV log, to evaluate the heading controller.

It publishes the setpoints of `sequence` on /guidance/heading, one after the other, and records at 20 Hz:
time, setpoint, the heading measured by the controller (/gnc/heading), the true heading and the fin bias
(/gnc/tail_bias). The true heading comes from the orientation of /imu/data, which is perfect in the simulation:
it is used here only to evaluate, never by the controller. At the end of the sequence the node writes the CSV
and stops.

    ros2 run gnc_control heading_test --ros-args -p sequence:="0:8,90:12,180:12,270:12,0:12" -p output:=square.csv

`sequence`: comma-separated "heading_deg:duration_s" pairs, headings clockwise from true north.
"""
import csv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from .heading import heading_from_quaternion


def parse_sequence(text):
    """'0:8,90:12' → [(0.0, 8.0), (90.0, 12.0)]"""
    steps = []
    for item in text.split(','):
        heading, duration = item.split(':')
        steps.append((float(heading), float(duration)))
    return steps


class HeadingTestNode(Node):
    def __init__(self):
        super().__init__('heading_test')
        self.sequence = parse_sequence(self.declare_parameter('sequence', '0:10,90:15,0:15').value)
        self.output = self.declare_parameter('output', 'heading_test.csv').value
        self.measured = self.true = self.bias = float('nan')
        self.create_subscription(Float64, 'gnc/heading', lambda m: setattr(self, 'measured', m.data), 10)
        self.create_subscription(Float64, 'gnc/tail_bias', lambda m: setattr(self, 'bias', m.data), 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, 10)
        self.pub = self.create_publisher(Float64, 'guidance/heading', 10)
        self.rows = []
        self.start = None
        self.done = False
        self.create_timer(0.05, self.step)

    def on_imu(self, msg):
        q = msg.orientation
        self.true = heading_from_quaternion(q.x, q.y, q.z, q.w)

    def step(self):
        if self.true != self.true:                                       # NaN: wait for the simulator
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start is None:
            self.start = now
            self.get_logger().info(f'sequence started: {self.sequence}')
        t = now - self.start
        elapsed = 0.0
        for heading, duration in self.sequence:                         # current setpoint
            elapsed += duration
            if t < elapsed:
                break
        else:
            self.finish()
            return
        self.pub.publish(Float64(data=heading))
        self.rows.append((round(t, 3), heading, self.measured, self.true, self.bias))

    def finish(self):
        if self.done:
            return
        self.done = True
        with open(self.output, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'setpoint_deg', 'measured_deg', 'true_deg', 'bias_deg'])
            w.writerows(self.rows)
        self.get_logger().info(f'{len(self.rows)} rows written to {self.output}')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = HeadingTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
