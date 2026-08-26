import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion


class LocalisationNode(Node):
    """
    Fuses GPS + IMU into a single pose estimate using a complementary filter.

    SIMPLIFICATION (flag honestly in interview): this is NOT a full EKF.
    A real localisation stack (e.g. robot_localization's ekf_node, which is
    the standard ROS2 package for this) would run proper covariance
    propagation and a time-varying Kalman gain, and would convert lat/lon
    into a local ENU/UTM frame before fusing. Here we:
      - convert GPS lat/lon into a simple local XY frame using a flat-earth
        approximation (fine over the small area a mower operates in, would
        NOT be fine for large-scale navigation)
      - blend GPS-derived position with IMU-integrated position using a
        FIXED blend weight (alpha) rather than a proper Kalman gain that
        adapts based on measurement confidence
    The reasoning behind fusing at all (GPS = accurate but noisy/low-rate/
    can dropout; IMU = high-rate but drifts unbounded) is the same
    reasoning a real EKF is built on — this just implements it with
    simpler math so it's fully explainable.
    """

    def __init__(self):
        super().__init__('localisation_node')

        # Flat-earth approximation constants (metres per degree, ~UK latitude)
        self.origin_lat = 51.5
        self.origin_lon = -0.5
        self.meters_per_deg_lat = 111320.0
        self.meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.origin_lat))

        # Fused state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # IMU-only dead-reckoning estimate (drifts over time — that's the point)
        self.imu_x = 0.0
        self.imu_y = 0.0
        self.imu_yaw = 0.0
        self.last_imu_time = None

        # Complementary filter blend weight: how much we trust IMU-integrated
        # motion vs. snapping to the latest GPS fix. Higher = smoother but
        # slower to correct GPS drift; lower = jumpier but tracks GPS closer.
        self.alpha = 0.85

        self.latest_gps = None  # (x, y) in local frame

        self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.pose_pub = self.create_publisher(Odometry, 'localisation/odom', 10)

        self.get_logger().info('Localisation node started (complementary filter)')

    def gps_to_local_xy(self, lat, lon):
        x = (lon - self.origin_lon) * self.meters_per_deg_lon
        y = (lat - self.origin_lat) * self.meters_per_deg_lat
        return x, y

    def gps_callback(self, msg: NavSatFix):
        gx, gy = self.gps_to_local_xy(msg.latitude, msg.longitude)
        self.latest_gps = (gx, gy)

        # Complementary blend: mostly trust IMU-integrated motion for smoothness,
        # but pull toward the GPS fix to correct drift.
        self.x = self.alpha * self.x + (1.0 - self.alpha) * gx
        self.y = self.alpha * self.y + (1.0 - self.alpha) * gy

        self.publish_pose()

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.last_imu_time is None:
            self.last_imu_time = now
            return

        dt = now - self.last_imu_time
        self.last_imu_time = now

        # Integrate yaw rate
        self.yaw += msg.angular_velocity.z * dt
        self.imu_yaw += msg.angular_velocity.z * dt

        # Very simplified dead-reckoning: integrate linear acceleration into
        # a small position delta. This is NOT proper double-integration with
        # velocity state (a real system would maintain velocity explicitly,
        # and IMU accel-based position integration drifts fast even then) —
        # simplified here to keep the filter easy to explain.
        dx = msg.linear_acceleration.x * dt * dt
        dy = msg.linear_acceleration.y * dt * dt
        self.x += dx * math.cos(self.yaw) - dy * math.sin(self.yaw)
        self.y += dx * math.sin(self.yaw) + dy * math.cos(self.yaw)

        self.publish_pose()

    def publish_pose(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        msg.pose.pose.orientation = Quaternion(
            z=math.sin(self.yaw / 2.0),
            w=math.cos(self.yaw / 2.0),
        )

        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalisationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
