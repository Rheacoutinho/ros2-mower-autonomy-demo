import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, Range
from nav_msgs.msg import Odometry


class SensorNode(Node):
    """
    Simulates a GPS + IMU + single-beam range sensor.

    RANGE SENSOR DESIGN: the range reading is now computed as a real
    line-of-sight distance from the vehicle's actual position, along its
    actual heading, to the nearest wall cell -- using the SAME wall
    geometry mower_planner's A* search avoids. This replaces an earlier
    version that used a pure sine wave with no connection to the vehicle's
    real position, which made the safety monitor's stop/clear behaviour
    look arbitrary rather than proximity-based.

    HONEST LIMITATION (flag explicitly): this is still not a real LiDAR
    or ultrasonic sensor. There's no true field-of-view cone, no multi-beam
    scan, no reflection physics, no sensor noise model -- just a single
    ray-marched line-of-sight check along the vehicle's heading. It IS,
    however, now genuinely tied to real geometry and real vehicle pose,
    which a sine wave was not.

    ALSO WORTH FLAGGING: the wall geometry here is duplicated from
    mower_planner's build_synthetic_grid(), hardcoded independently in
    two different nodes. In a real system this is exactly the kind of
    thing that causes bugs -- two components silently disagreeing about
    the map because nobody kept them in sync. A real system would have a
    single shared map source (e.g. a published nav_msgs/OccupancyGrid or a
    shared parameter/config) that every consumer reads, not duplicated
    hardcoded copies. Left duplicated here to avoid adding a cross-package
    dependency for a small demo, but worth naming as a design smell if
    asked.
    """

    def __init__(self):
        super().__init__('sensor_node')

        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.range_pub = self.create_publisher(Range, 'obstacle/range', 10)

        self.create_subscription(Odometry, 'vehicle/odom', self.odom_callback, 10)

        self.start_time = time.time()

        self.origin_lat = 51.5
        self.origin_lon = -0.5

        # Vehicle pose, updated from vehicle_model_node's odom. None until
        # the first message arrives.
        self.vehicle_x = None
        self.vehicle_y = None
        self.vehicle_yaw = 0.0

        # Must match mower_planner's build_synthetic_grid() exactly --
        # see the class docstring about this duplication.
        self.grid_width = 10
        self.grid_height = 10
        self.resolution = 1.0
        self.grid = self.build_synthetic_grid()

        self.range_min = 0.02
        self.range_max = 4.0
        self.ray_step = 0.05  # metres per ray-march step

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_all)

        self.get_logger().info(
            'Sensor node started, publishing at 10 Hz '
            '(range now computed from real vehicle pose vs. wall geometry)')

    def build_synthetic_grid(self):
        grid = [[0 for _ in range(self.grid_width)] for _ in range(self.grid_height)]
        for y in range(0, 7):
            grid[y][5] = 1
        return grid

    def odom_callback(self, msg: Odometry):
        self.vehicle_x = msg.pose.pose.position.x
        self.vehicle_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.vehicle_yaw = 2.0 * math.atan2(q.z, q.w)

    def is_wall(self, x, y):
        gx = round(x / self.resolution)
        gy = round(y / self.resolution)
        if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
            return self.grid[gy][gx] == 1
        return False

    def compute_range_to_wall(self):
        """Ray-march forward from the vehicle's real pose until a wall
        cell is hit, or range_max is reached."""
        if self.vehicle_x is None:
            # No odom yet -- report max range (safe/clear) rather than a
            # fabricated close reading, so the safety monitor doesn't
            # falsely trigger before the vehicle model has even started.
            return self.range_max

        steps = int(self.range_max / self.ray_step)
        for i in range(1, steps + 1):
            d = i * self.ray_step
            px = self.vehicle_x + d * math.cos(self.vehicle_yaw)
            py = self.vehicle_y + d * math.sin(self.vehicle_yaw)
            if self.is_wall(px, py):
                return max(self.range_min, d)
        return self.range_max

    def publish_all(self):
        t = time.time() - self.start_time
        self.publish_gps(t)
        self.publish_imu(t)
        self.publish_range()

    def publish_gps(self, t):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.latitude = self.origin_lat + 0.0001 * math.sin(t * 0.1)
        msg.longitude = self.origin_lon + 0.0001 * math.cos(t * 0.1)
        msg.altitude = 50.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        msg.position_covariance = [2.0, 0.0, 0.0,
                                    0.0, 2.0, 0.0,
                                    0.0, 0.0, 4.0]
        self.gps_pub.publish(msg)

    def publish_imu(self, t):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        yaw = 0.1 * t
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)
        msg.angular_velocity.z = 0.1
        msg.linear_acceleration.x = 0.05 * math.sin(t)
        msg.linear_acceleration.y = 0.05 * math.cos(t)
        msg.linear_acceleration.z = 9.81
        self.imu_pub.publish(msg)

    def publish_range(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'range_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.5
        msg.min_range = self.range_min
        msg.max_range = self.range_max
        msg.range = self.compute_range_to_wall()
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
