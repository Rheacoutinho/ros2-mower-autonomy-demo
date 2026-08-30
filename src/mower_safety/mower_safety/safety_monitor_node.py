import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger


class SafetyMonitorNode(Node):
    """
    MANUAL RESET / OVERRIDE: once the vehicle stops facing an obstacle, its
    heading stops changing too (control_node publishes zero Twist while
    gated), so a heading-based range reading never improves on its own --
    the vehicle can get permanently stuck. Rather than silently
    auto-clearing near an obstacle (dangerous), this exposes an explicit
    operator-callable /safety/reset service that opens a SHORT,
    TIME-LIMITED override window (default 3s) during which gated commands
    pass through so the vehicle can maneuver clear. Once the window
    expires, normal safety evaluation resumes automatically and will
    immediately re-engage the stop if still too close -- this never
    disables the safety function outright, only bypasses it briefly and
    on an explicit, logged operator action.
    """

    def __init__(self):
        super().__init__('safety_monitor_node')

        self.declare_parameter('min_safe_range', 0.8)
        self.declare_parameter('override_duration_sec', 3.0)
        self.min_safe_range = self.get_parameter('min_safe_range').value
        self.override_duration_sec = self.get_parameter('override_duration_sec').value

        self.safe_stop_active = False
        self.latest_range = None
        self.override_until = 0.0  # monotonic time; expired by default

        self.create_subscription(Range, 'obstacle/range', self.range_callback, 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_vel_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'safety/status', 10)
        self.stop_active_pub = self.create_publisher(Bool, 'safety/stop_active', 10)
        self.marker_pub = self.create_publisher(Marker, 'safety/status_marker', 10)

        self.reset_srv = self.create_service(Trigger, 'safety/reset', self.handle_reset)

        self.status_timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info(
            f'Safety monitor started, min_safe_range={self.min_safe_range}m, '
            f'override_duration={self.override_duration_sec}s')

    def override_active(self):
        return time.monotonic() < self.override_until

    def range_callback(self, msg):
        self.latest_range = msg.range
        was_active = self.safe_stop_active
        hazard_present = self.latest_range < self.min_safe_range
        self.safe_stop_active = hazard_present and not self.override_active()

        if self.safe_stop_active and not was_active:
            self.get_logger().warn(
                f'SAFE-STOP TRIGGERED: range {self.latest_range:.2f}m '
                f'< threshold {self.min_safe_range}m')
        elif was_active and not self.safe_stop_active:
            self.get_logger().info(
                f'Safe-stop cleared: range {self.latest_range:.2f}m, '
                f'override_active={self.override_active()}')

    def cmd_vel_callback(self, msg):
        if self.safe_stop_active:
            self.cmd_pub.publish(Twist())
        else:
            self.cmd_pub.publish(msg)

    def handle_reset(self, request, response):
        if self.latest_range is None:
            response.success = False
            response.message = 'No range data yet, cannot arm override'
            return response

        self.override_until = time.monotonic() + self.override_duration_sec
        was_active = self.safe_stop_active
        self.safe_stop_active = False

        self.get_logger().warn(
            f'MANUAL SAFETY OVERRIDE engaged for {self.override_duration_sec}s '
            f'(was_active={was_active}, range={self.latest_range:.2f}m). '
            f'Vehicle may move even though an obstacle may still be near; '
            f'operator responsibility for the override window.')

        response.success = True
        response.message = (
            f'Override engaged for {self.override_duration_sec}s. '
            f'Safety will automatically re-arm after the window.')
        return response

    def publish_status(self):
        status_msg = String()
        if self.latest_range is None:
            status_msg.data = 'NO_DATA'
        elif self.override_active():
            status_msg.data = f'OVERRIDE_ACTIVE (range={self.latest_range:.2f}m)'
        elif self.safe_stop_active:
            status_msg.data = f'UNSAFE_STOPPED (range={self.latest_range:.2f}m)'
        else:
            status_msg.data = f'SAFE (range={self.latest_range:.2f}m)'
        self.status_pub.publish(status_msg)

        stop_msg = Bool()
        stop_msg.data = self.safe_stop_active
        self.stop_active_pub.publish(stop_msg)

        self.publish_status_marker()

    def publish_status_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'safety_status'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = -1.0
        marker.pose.position.y = -1.0
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        if self.override_active():
            marker.color.r = 1.0
            marker.color.g = 1.0  # yellow: override window active
        elif self.safe_stop_active:
            marker.color.r = 1.0
            marker.color.g = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
