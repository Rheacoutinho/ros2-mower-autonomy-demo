import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


class SafetyMonitorNode(Node):
    """
    Independent safety monitor implementing a software safe-stop.

    ARCHITECTURE NOTE (interview talking point): this node sits
    DOWNSTREAM of the control node, intercepting cmd_vel on one topic
    and republishing a safety-gated version on another. This means a
    bug in planning/control logic cannot silently bypass the safety
    check -- the safety function is architecturally independent from
    the primary autonomy logic, which is a core functional-safety
    design principle (defence in depth / independent safety channel).

    HONEST LIMITATION (flag explicitly): this is a SOFTWARE-ONLY
    safe-stop running in the same ROS2 graph as everything else. If
    the ROS2 process, this node's process, or the underlying OS
    crashes or hangs, this safety function stops working too -- which
    is NOT acceptable for a real safety-critical function. A real
    system needs an independent hardware safety channel (safety-rated
    PLC, hardwired e-stop, watchdog that cuts power at the actuator
    level) that doesn't depend on software health. This node
    demonstrates the SOFTWARE safety-logic layer only -- one part of
    a real safety architecture, not the whole thing. This maps
    directly to the JD's HARA / safety case activities: a real HARA
    would identify this exact gap (single software channel = single
    point of failure) and require an independent hardware mitigation.
    """

    def __init__(self):
        super().__init__('safety_monitor_node')

        # Below this range (metres), trigger safe-stop.
        self.declare_parameter('min_safe_range', 0.8)
        self.min_safe_range = self.get_parameter('min_safe_range').value

        self.safe_stop_active = False
        self.latest_range = None

        self.create_subscription(Range, 'obstacle/range', self.range_callback, 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_vel_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'safety/status', 10)
        self.stop_active_pub = self.create_publisher(Bool, 'safety/stop_active', 10)

        # Publish status at a fixed rate too, so it's visible even with no obstacle changes
        self.status_timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info(
            f'Safety monitor started, min_safe_range={self.min_safe_range}m')

    def range_callback(self, msg: Range):
        self.latest_range = msg.range

        was_active = self.safe_stop_active
        self.safe_stop_active = self.latest_range < self.min_safe_range

        if self.safe_stop_active and not was_active:
            self.get_logger().warn(
                f'SAFE-STOP TRIGGERED: range {self.latest_range:.2f}m '
                f'< threshold {self.min_safe_range}m')
        elif was_active and not self.safe_stop_active:
            self.get_logger().info(
                f'Safe-stop cleared: range {self.latest_range:.2f}m is clear')

    def cmd_vel_callback(self, msg: Twist):
        """Gate incoming velocity commands based on safety state."""
        if self.safe_stop_active:
            safe_msg = Twist()  # zero velocity, safety overrides everything
            self.cmd_pub.publish(safe_msg)
        else:
            self.cmd_pub.publish(msg)  # pass through unmodified

    def publish_status(self):
        status_msg = String()
        if self.latest_range is None:
            status_msg.data = 'NO_DATA'
        elif self.safe_stop_active:
            status_msg.data = f'UNSAFE_STOPPED (range={self.latest_range:.2f}m)'
        else:
            status_msg.data = f'SAFE (range={self.latest_range:.2f}m)'
        self.status_pub.publish(status_msg)

        stop_msg = Bool()
        stop_msg.data = self.safe_stop_active
        self.stop_active_pub.publish(stop_msg)


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
