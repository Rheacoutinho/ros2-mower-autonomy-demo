import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist


class ControlNode(Node):
    """
    Simple proportional waypoint-following controller.

    SIMPLIFICATION (flag honestly): a real path-tracking controller would
    typically be pure pursuit, a Stanley controller, or MPC -- accounting
    properly for lookahead distance, vehicle dynamics, and path curvature.
    This is a basic P-controller on heading error plus a fixed forward
    speed, advancing to the next waypoint once within a tolerance radius.
    Same *category* of problem (convert a planned path into actuator
    commands) and same feedback-loop structure, much simpler math.
    """

    def __init__(self):
        super().__init__('control_node')

        self.path = []
        self.current_waypoint_idx = 0
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0

        self.waypoint_tolerance = 0.5   # metres
        self.forward_speed = 0.5        # m/s
        self.heading_gain = 1.5         # proportional gain on heading error

        # Safety monitor can set this to stop commands (Phase 5 hooks in here)
        self.safe_stop_active = False

        self.create_subscription(Path, 'planner/path', self.path_callback, 10)
        self.create_subscription(Odometry, 'vehicle/odom', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)

        self.timer = self.create_timer(0.1, self.control_step)  # 10 Hz

        self.get_logger().info('Control node started (P-controller waypoint following)')

    def path_callback(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        # Only reset progress the first time a path arrives, so we don't
        # restart from waypoint 0 every time the planner republishes.
        if self.current_waypoint_idx >= len(self.path):
            self.current_waypoint_idx = 0

    def odom_callback(self, msg: Odometry):
        self.vehicle_x = msg.pose.pose.position.x
        self.vehicle_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.vehicle_yaw = 2.0 * math.atan2(q.z, q.w)

    def control_step(self):
        cmd = Twist()

        if self.safe_stop_active:
            self.cmd_pub.publish(cmd)  # zero Twist
            return

        if not self.path or self.current_waypoint_idx >= len(self.path):
            self.cmd_pub.publish(cmd)  # nothing to do, stay stopped
            return

        target_x, target_y = self.path[self.current_waypoint_idx]
        dx = target_x - self.vehicle_x
        dy = target_y - self.vehicle_y
        distance = math.hypot(dx, dy)

        if distance < self.waypoint_tolerance:
            self.current_waypoint_idx += 1
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_idx}/{len(self.path)}')
            self.cmd_pub.publish(cmd)
            return

        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - self.vehicle_yaw
        # Normalise to [-pi, pi]
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        cmd.linear.x = self.forward_speed
        cmd.angular.z = self.heading_gain * heading_error

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
