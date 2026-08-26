import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class VehicleModelNode(Node):
    """
    Simple 2D kinematic vehicle model standing in for the mower.

    SIMPLIFICATION (flag honestly): this is a pure kinematic integrator --
    no mass, no inertia, no wheel slip, no terrain interaction (grass,
    slopes -- all highly relevant for a real mower and exactly the kind
    of thing field trials would surface). Velocity commands are applied
    instantly and exactly. A real vehicle model (or real vehicle) would
    have significant dynamics between commanded and actual velocity.
    This is sufficient to demonstrate the control-loop *structure*
    (plan -> control -> actuation -> feedback), just not real-world
    vehicle behaviour.
    """

    def __init__(self):
        super().__init__('vehicle_model_node')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0       # current commanded linear velocity
        self.omega = 0.0   # current commanded angular velocity

        self.last_time = self.get_clock().now()

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'vehicle/odom', 10)
        self.marker_pub = self.create_publisher(Marker, 'vehicle/marker', 10)

        # Integrate + publish at 20 Hz regardless of when cmd_vel messages arrive,
        # so the vehicle keeps moving smoothly between control updates.
        self.timer = self.create_timer(0.05, self.step)

        self.get_logger().info('Vehicle model node started (2D kinematic)')

    def cmd_vel_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0:
            return

        # Basic unicycle kinematic model
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.omega * dt

        self.publish_odom(now)
        self.publish_marker(now)

    def publish_odom(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.omega
        self.odom_pub.publish(msg)

    def publish_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp.to_msg()
        marker.ns = 'mower'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.1
        marker.pose.orientation.z = math.sin(self.yaw / 2.0)
        marker.pose.orientation.w = math.cos(self.yaw / 2.0)
        marker.scale.x = 0.8
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.1
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
