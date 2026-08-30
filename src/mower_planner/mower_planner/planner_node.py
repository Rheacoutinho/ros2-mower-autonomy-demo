import heapq
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker


class PlannerNode(Node):
    """
    Simple grid-based A* planner over a synthetic occupancy grid.

    SIMPLIFICATION (flag honestly): a real mower planner would typically be
    a COVERAGE planner (cover the whole field, e.g. boustrophedon/back-and-
    forth sweep), built from a real surveyed field boundary or perception-
    derived map, not point-to-point A* over a hand-coded grid. A* is used
    here because it's the most universally recognisable planning algorithm
    to implement and explain clearly in limited time. The underlying search
    algorithm (A*) is the same class of algorithm real planners use (e.g.
    Nav2's NavFn/Smac planners are also grid-search-based); only the map
    input and the point-to-point (vs coverage) framing are simplified.
    """

    def __init__(self):
        super().__init__('planner_node')

        self.grid_width = 10
        self.grid_height = 10
        self.resolution = 1.0
        self.grid = self.build_synthetic_grid()

        self.start = (0, 0)
        self.goal = (9, 9)

        self.path_pub = self.create_publisher(Path, 'planner/path', 10)
        self.obstacle_marker_pub = self.create_publisher(
            Marker, 'planner/obstacle_marker', 10)

        self.timer = self.create_timer(2.0, self.plan_and_publish)

        self.get_logger().info('Planner node started (grid A*)')

    def build_synthetic_grid(self):
        grid = [[0 for _ in range(self.grid_width)] for _ in range(self.grid_height)]
        for y in range(0, 7):
            grid[y][5] = 1
        return grid

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def neighbors(self, cell):
        x, y = cell
        candidates = [
            (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),
            (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1), (x - 1, y + 1),
        ]
        result = []
        for nx, ny in candidates:
            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                if self.grid[ny][nx] == 0:
                    result.append((nx, ny))
        return result

    def astar(self, start, goal):
        open_set = [(0.0, start)]
        came_from = {}
        g_score = {start: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for neighbor in self.neighbors(current):
                step_cost = self.heuristic(current, neighbor)
                tentative_g = g_score[current] + step_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    def plan_and_publish(self):
        cell_path = self.astar(self.start, self.goal)

        self.publish_obstacle_marker()

        if cell_path is None:
            self.get_logger().warn('No path found from start to goal')
            return

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for (cx, cy) in cell_path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = cx * self.resolution
            pose.pose.position.y = cy * self.resolution
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def publish_obstacle_marker(self):
        """
        Renders the wall cells from self.grid as a CUBE_LIST, so the
        obstacle the A* path is routing around is actually visible in
        RViz2, not just an invisible input to the search.
        """
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'obstacles'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.resolution * 0.9
        marker.scale.y = self.resolution * 0.9
        marker.scale.z = 0.5
        marker.color.r = 0.8
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        for gy in range(self.grid_height):
            for gx in range(self.grid_width):
                if self.grid[gy][gx] == 1:
                    p = Point()
                    p.x = gx * self.resolution
                    p.y = gy * self.resolution
                    p.z = 0.25
                    marker.points.append(p)

        self.obstacle_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
