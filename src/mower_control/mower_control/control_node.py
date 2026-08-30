import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist


class ControlNode(Node):
    """
    Proportional waypoint-following controller.

    LOOPING BEHAVIOUR: once the vehicle reaches the end of the A* path, it
    reverses direction and walks the SAME path backward, rather than
    jumping back to waypoint 0 directly. This matters because the
    controller has no independent obstacle awareness of its own -- it only
    avoids the wall by following the pre-computed A* waypoint sequence in
    order. Snapping straight back to a distant waypoint would produce a
    straight-line move that ignores the obstacle entirely. Reversing the
    same path guarantees every leg, forward or backward, stays on a route
    A* already verified is obstacle-free.
    """

    def __init__(self):
        super().__init__('control_node')

        self.path = []
        self.current_waypoint_idx = 0
        self.direction = 1  # 1 = forward through path, -1 = walking it backward
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0

        self.waypoint_tolerance = 0.5
        self.forward_speed = 0.5
        self.heading_gain = 1.5
        self.safe_stop_active = False

        self.create_subscription(Path, 'planner/path', self.path_callback, 10)
        self.create_subscription(Odometry, 'vehicle/odom', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)

        self.timer = self.create_timer(0.1, self.control_step)

        self.get_logger().info(
            'Control node started (P-controller, ping-pong waypoint following)')

    def path_callback(self, msg: Path):
        new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        # Only reset progress if the path content actually changed (e.g. a
        # genuinely new plan). A same-content republish (which happens every
        # 2s just so late subscribers like RViz2 see it) should NOT disturb
        # in-progress navigation.
        if new_path != self.path:
            self.path = new_path
            self.current_waypoint_idx = 0
            self.direction = 1

    def odom_callback(self, msg: Odometry):
        self.vehicle_x = msg.pose.pose.position.x
        self.vehicle_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.vehicle_yaw = 2.0 * math.atan2(q.z, q.w)

    def control_step(self):
        cmd = Twist()

        if self.safe_stop_active:
            self.cmd_pub.publish(cmd)
            return

        if not self.path:
            self.cmd_pub.publish(cmd)
            return

        target_x, target_y = self.path[self.current_waypoint_idx]
        dx = target_x - self.vehicle_x
        dy = target_y - self.vehicle_y
        distance = math.hypot(dx, dy)

        if distance < self.waypoint_tolerance:
            next_idx = self.current_waypoint_idx + self.direction

            if next_idx >= len(self.path):
                # Reached the end going forward -- reverse and re-walk the
                # same path backward, staying on the verified route.
                self.direction = -1
                self.current_waypoint_idx = len(self.path) - 2 \
                    if len(self.path) > 1 else 0
                self.get_logger().info(
                    'Reached path end, reversing direction (ping-pong)')
            elif next_idx < 0:
                # Reached the start going backward -- reverse again.
                self.direction = 1
                self.current_waypoint_idx = 1 if len(self.path) > 1 else 0
                self.get_logger().info(
                    'Reached path start, reversing direction (ping-pong)')
            else:
                self.current_waypoint_idx = next_idx
                self.get_logger().info(
                    f'Reached waypoint {self.current_waypoint_idx}/{len(self.path) - 1} '
                    f'(direction={"forward" if self.direction == 1 else "backward"})')

            self.cmd_pub.publish(cmd)
            return

        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - self.vehicle_yaw
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
