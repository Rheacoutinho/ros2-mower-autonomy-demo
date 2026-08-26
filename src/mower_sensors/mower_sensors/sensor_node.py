import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, Range


class SensorNode(Node):
    """
    Simulates a GPS + IMU + single-beam range sensor.
    Publishes at 10 Hz. Values are synthetic (no hardware, no physics sim) —
    this stands in for what would be real driver nodes wrapping GPS/IMU/LiDAR
    hardware, but uses the real ROS2 sensor message types so anything
    downstream (localisation, safety monitor) sees exactly the same
    interface a real driver would produce.
    """

    def __init__(self):
        super().__init__('sensor_node')

        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.range_pub = self.create_publisher(Range, 'obstacle/range', 10)

        self.start_time = time.time()

        # Fake origin — roughly a UK field location, doesn't matter exactly
        self.origin_lat = 51.5
        self.origin_lon = -0.5

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_all)

        self.get_logger().info('Sensor node started, publishing at 10 Hz')

    def publish_all(self):
        t = time.time() - self.start_time
        self.publish_gps(t)
        self.publish_imu(t)
        self.publish_range(t)

    def publish_gps(self, t):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        # Slow circular drift so localisation later has something to fuse
        msg.latitude = self.origin_lat + 0.0001 * math.sin(t * 0.1)
        msg.longitude = self.origin_lon + 0.0001 * math.cos(t * 0.1)
        msg.altitude = 50.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        # Rough GPS uncertainty in metres^2, diagonal covariance
        msg.position_covariance = [2.0, 0.0, 0.0,
                                    0.0, 2.0, 0.0,
                                    0.0, 0.0, 4.0]
        self.gps_pub.publish(msg)

    def publish_imu(self, t):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        # Fake slow yaw rotation, small noise on the rest
        yaw = 0.1 * t
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)
        msg.angular_velocity.z = 0.1
        msg.linear_acceleration.x = 0.05 * math.sin(t)
        msg.linear_acceleration.y = 0.05 * math.cos(t)
        msg.linear_acceleration.z = 9.81
        self.imu_pub.publish(msg)

    def publish_range(self, t):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'range_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.5
        msg.min_range = 0.02
        msg.max_range = 4.0
        # Oscillate between far (safe) and occasionally close (triggers safety later)
        distance = 2.0 + 1.5 * math.sin(t * 0.3)
        msg.range = max(msg.min_range, distance)
        self.range_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
