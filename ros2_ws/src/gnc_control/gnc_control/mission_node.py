"""ROS 2 node: runs a mission from a file (GNC step 5, see Docs/GNC.md, section 10).

Subscriptions:
  /state/odom        nav_msgs/Odometry   position and depth from the navigation
  /teleop/active     std_msgs/Bool       an operator has taken the robot: the mission ends and does not resume
Publications:
  /guidance/heading  std_msgs/Float64    heading setpoint [deg, clockwise from true north]
  /guidance/depth    std_msgs/Float64    depth setpoint [m]
  /guidance/swim     std_msgs/Bool       False: the tail must not flap
  /mission/status    std_msgs/String     phase, step and reason

The logic is in mission_plan.py.
"""
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String

from .mission_plan import MissionRunner, parse_mission
from .shutdown import install_stop_handlers


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission')
        p = self.declare_parameter
        path = p('mission_file', '').value                               # YAML file of the mission
        self.state_timeout = p('state_timeout_s', 1.0).value             # no state for this long: state lost [s]
        self.teleop_timeout = p('teleop_timeout_s', 0.5).value           # [s]
        with open(path, encoding='utf-8') as f:
            self.runner = MissionRunner(parse_mission(yaml.safe_load(f)))

        self.north = self.west = self.depth = None
        self.state_time = None                                           # [s]
        self.teleop_time = None                                          # [s]
        self.last_status = ''

        self.create_subscription(Odometry, 'state/odom', self.on_odom, 10)
        self.create_subscription(Bool, 'teleop/active', self.on_teleop, 10)
        self.pub_heading = self.create_publisher(Float64, 'guidance/heading', 10)
        self.pub_depth = self.create_publisher(Float64, 'guidance/depth', 10)
        self.pub_swim = self.create_publisher(Bool, 'guidance/swim', 10)
        self.pub_status = self.create_publisher(String, 'mission/status', 10)
        self.create_timer(0.1, self.step)
        self.get_logger().info(f'mission loaded from {path}: {len(self.runner.mission.steps)} steps')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9                 # [s]

    def on_odom(self, msg):
        position = msg.pose.pose.position
        self.north, self.west, self.depth = position.x, position.y, -position.z
        self.state_time = self.now()

    def on_teleop(self, msg):
        if msg.data:
            self.teleop_time = self.now()

    def step(self):
        t = self.now()
        fresh = self.state_time is not None and t - self.state_time <= self.state_timeout
        teleop = self.teleop_time is not None and t - self.teleop_time <= self.teleop_timeout
        out = self.runner.update(t, self.north if fresh else None, self.west if fresh else None,
                                 self.depth if fresh else None, teleop)
        if out.heading is not None:
            self.pub_heading.publish(Float64(data=float(out.heading)))
        if out.depth is not None:
            self.pub_depth.publish(Float64(data=float(out.depth)))
        self.pub_swim.publish(Bool(data=out.swim))

        status = f'{out.phase} step={out.step}/{len(self.runner.mission.steps)} {out.reason}'.strip()
        self.pub_status.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_stop_handlers()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub_swim.publish(Bool(data=False))                      # the last message stops the tail
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
