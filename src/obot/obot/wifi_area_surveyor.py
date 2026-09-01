#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Optional, Set, Tuple

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


@dataclass
class GridGoal:
    gx: int
    gy: int
    x: float
    y: float


class WifiAreaSurveyor(Node):
    def __init__(self) -> None:
        super().__init__('wifi_area_surveyor')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('nav_action_name', '/navigate_to_pose')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('boundary_marker_topic', '/wifi/survey_boundary')
        self.declare_parameter('use_interactive_bounds', True)
        self.declare_parameter('use_polygon_bounds', True)
        self.declare_parameter('polygon_close_distance_m', 0.5)
        self.declare_parameter('stride_cells', 8)
        self.declare_parameter('free_threshold', 20)
        self.declare_parameter('goal_tolerance_m', 0.35)

        # Optional area bounds in meters. Leave as NaN to cover all free known space.
        self.declare_parameter('min_x', float('nan'))
        self.declare_parameter('max_x', float('nan'))
        self.declare_parameter('min_y', float('nan'))
        self.declare_parameter('max_y', float('nan'))

        self.map_topic = str(self.get_parameter('map_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.robot_base_frame = str(self.get_parameter('robot_base_frame').value)
        nav_action_name = str(self.get_parameter('nav_action_name').value)
        self.clicked_point_topic = str(self.get_parameter('clicked_point_topic').value)
        self.boundary_marker_topic = str(self.get_parameter('boundary_marker_topic').value)
        self.use_interactive_bounds = bool(self.get_parameter('use_interactive_bounds').value)
        self.use_polygon_bounds = bool(self.get_parameter('use_polygon_bounds').value)
        self.polygon_close_distance_m = float(self.get_parameter('polygon_close_distance_m').value)
        self.stride_cells = max(1, int(self.get_parameter('stride_cells').value))
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.goal_tolerance_m = float(self.get_parameter('goal_tolerance_m').value)

        self.min_x = float(self.get_parameter('min_x').value)
        self.max_x = float(self.get_parameter('max_x').value)
        self.min_y = float(self.get_parameter('min_y').value)
        self.max_y = float(self.get_parameter('max_y').value)

        self.map_msg: Optional[OccupancyGrid] = None
        self.goals: List[GridGoal] = []
        self.next_goal_idx = 0
        self.active_goal: Optional[GridGoal] = None
        self.visited: Set[Tuple[int, int]] = set()
        self.goal_in_flight = False
        self.map_replanned = False
        self.pending_boundary_click: Optional[Tuple[float, float]] = None
        self.interactive_bounds_ready = False
        self.polygon_vertices: List[Tuple[float, float]] = []
        self.polygon_closed = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, nav_action_name)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 10)
        self.create_subscription(PointStamped, self.clicked_point_topic, self._clicked_point_cb, 10)
        self.boundary_pub = self.create_publisher(Marker, self.boundary_marker_topic, 5)
        self.create_timer(1.0, self._tick)
        self.create_timer(0.5, self._publish_boundary_marker)

        self.get_logger().info(
            f'WifiAreaSurveyor started: map_topic={self.map_topic}, nav_action={nav_action_name}, '
            f'stride_cells={self.stride_cells}, interactive_bounds={self.use_interactive_bounds}, '
            f'polygon_bounds={self.use_polygon_bounds}'
        )

    def _clicked_point_cb(self, msg: PointStamped) -> None:
        if not self.use_interactive_bounds:
            return
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f'Ignoring clicked point frame {msg.header.frame_id}; expected {self.map_frame}'
            )
            return

        x = float(msg.point.x)
        y = float(msg.point.y)

        if self.use_polygon_bounds:
            self._handle_polygon_click(x, y)
            return

        if self.pending_boundary_click is None:
            self.pending_boundary_click = (x, y)
            self.get_logger().info(
                f'Boundary corner 1 set at ({x:.2f}, {y:.2f}). Click second corner to apply area.'
            )
            return

        x1, y1 = self.pending_boundary_click
        self.pending_boundary_click = None

        self.min_x = min(x1, x)
        self.max_x = max(x1, x)
        self.min_y = min(y1, y)
        self.max_y = max(y1, y)
        self.interactive_bounds_ready = True
        self.map_replanned = True

        self.get_logger().info(
            'Interactive boundary updated: '
            f'x=[{self.min_x:.2f},{self.max_x:.2f}] y=[{self.min_y:.2f},{self.max_y:.2f}]'
        )

    def _handle_polygon_click(self, x: float, y: float) -> None:
        if self.polygon_closed:
            # Start a new polygon after completion.
            self.polygon_vertices = [(x, y)]
            self.polygon_closed = False
            self.interactive_bounds_ready = False
            self.map_replanned = False
            self.get_logger().info(
                f'Polygon reset. Vertex 1 set at ({x:.2f}, {y:.2f}). Add more vertices to define area.'
            )
            return

        if len(self.polygon_vertices) >= 3:
            fx, fy = self.polygon_vertices[0]
            dist = math.hypot(x - fx, y - fy)
            if dist <= max(0.05, self.polygon_close_distance_m):
                self.polygon_closed = True
                self.interactive_bounds_ready = True
                self.map_replanned = True
                self.get_logger().info(
                    f'Polygon boundary closed with {len(self.polygon_vertices)} vertices. Replanning now.'
                )
                return

        self.polygon_vertices.append((x, y))
        self.get_logger().info(
            f'Polygon vertex {len(self.polygon_vertices)} added at ({x:.2f}, {y:.2f}). '
            f'Click near first vertex to close.'
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_replanned = True

    def _tick(self) -> None:
        if self.map_msg is None:
            return

        if self.map_replanned and not self.goal_in_flight:
            self.goals = self._build_coverage_goals(self.map_msg)
            self.next_goal_idx = 0
            self.active_goal = None
            self.visited.clear()
            self.map_replanned = False
            self.get_logger().info(f'Planned {len(self.goals)} coverage goals from current map')

        if self.goal_in_flight:
            return

        goal = self._pick_next_goal()
        if goal is None:
            self.get_logger().info('Coverage complete: no remaining reachable survey goals')
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().warn('NavigateToPose action server not available yet')
            return

        self._send_goal(goal)

    def _pick_next_goal(self) -> Optional[GridGoal]:
        if not self.goals:
            return None

        robot_xy = self._robot_xy()
        if robot_xy is None:
            # Fall back to sequence order when TF is unavailable.
            while self.next_goal_idx < len(self.goals):
                goal = self.goals[self.next_goal_idx]
                self.next_goal_idx += 1
                if (goal.gx, goal.gy) not in self.visited:
                    return goal
            return None

        # Choose nearest remaining goal for robust completion in fragmented spaces.
        rx, ry = robot_xy
        candidates = [g for g in self.goals if (g.gx, g.gy) not in self.visited]
        if not candidates:
            return None
        return min(candidates, key=lambda g: (g.x - rx) ** 2 + (g.y - ry) ** 2)

    def _robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except TransformException:
            return None

    def _send_goal(self, grid_goal: GridGoal) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = grid_goal.x
        pose.pose.position.y = grid_goal.y
        pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.goal_in_flight = True
        self.active_goal = grid_goal
        self.get_logger().info(
            f'Sending coverage goal cell=({grid_goal.gx},{grid_goal.gy}) '
            f'xy=({grid_goal.x:.2f},{grid_goal.y:.2f})'
        )

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Coverage goal rejected by navigator')
            if self.active_goal is not None:
                self.visited.add((self.active_goal.gx, self.active_goal.gy))
            self.goal_in_flight = False
            self.active_goal = None
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future) -> None:
        reached = False
        try:
            result = future.result()
            status = result.status
            # STATUS_SUCCEEDED is value 4 in action_msgs/GoalStatus.
            reached = status == 4
        except Exception as exc:
            self.get_logger().warn(f'Coverage goal result retrieval failed: {exc}')

        if self.active_goal is not None:
            self.visited.add((self.active_goal.gx, self.active_goal.gy))
            if reached:
                self.get_logger().info(
                    f'Reached coverage goal cell=({self.active_goal.gx},{self.active_goal.gy})'
                )
            else:
                self.get_logger().warn(
                    f'Navigator failed goal cell=({self.active_goal.gx},{self.active_goal.gy}), skipping'
                )

        self.goal_in_flight = False
        self.active_goal = None

    def _in_bounds(self, x: float, y: float) -> bool:
        if self.use_interactive_bounds and self.use_polygon_bounds and self.polygon_closed:
            return self._point_in_polygon(x, y, self.polygon_vertices)

        if not math.isnan(self.min_x) and x < self.min_x:
            return False
        if not math.isnan(self.max_x) and x > self.max_x:
            return False
        if not math.isnan(self.min_y) and y < self.min_y:
            return False
        if not math.isnan(self.max_y) and y > self.max_y:
            return False
        return True

    @staticmethod
    def _point_in_polygon(x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
        inside = False
        n = len(polygon)
        if n < 3:
            return False

        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            intersects = ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
            )
            if intersects:
                inside = not inside
            j = i
        return inside

    def _publish_boundary_marker(self) -> None:
        if self.use_polygon_bounds:
            self._publish_polygon_marker()
            return

        if not self.interactive_bounds_ready:
            self._clear_boundary_marker()
            return

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'wifi_survey_boundary'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.a = 0.9
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0

        corners = [
            (self.min_x, self.min_y),
            (self.max_x, self.min_y),
            (self.max_x, self.max_y),
            (self.min_x, self.max_y),
            (self.min_x, self.min_y),
        ]

        for cx, cy in corners:
            p = Point()
            p.x = float(cx)
            p.y = float(cy)
            p.z = 0.02
            marker.points.append(p)

        self.boundary_pub.publish(marker)

    def _publish_polygon_marker(self) -> None:
        if not self.polygon_vertices:
            self._clear_boundary_marker()
            return

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'wifi_survey_boundary'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.a = 0.95
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0

        points = list(self.polygon_vertices)
        if self.polygon_closed:
            points.append(self.polygon_vertices[0])

        for cx, cy in points:
            p = Point()
            p.x = float(cx)
            p.y = float(cy)
            p.z = 0.02
            marker.points.append(p)

        self.boundary_pub.publish(marker)

    def _clear_boundary_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'wifi_survey_boundary'
        marker.id = 1
        marker.action = Marker.DELETE
        self.boundary_pub.publish(marker)

    def _build_coverage_goals(self, map_msg: OccupancyGrid) -> List[GridGoal]:
        info = map_msg.info
        width = info.width
        height = info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        data = map_msg.data

        goals: List[GridGoal] = []
        y_values = list(range(0, height, self.stride_cells))

        for row_i, gy in enumerate(y_values):
            x_values = list(range(0, width, self.stride_cells))
            if row_i % 2 == 1:
                x_values.reverse()

            for gx in x_values:
                idx = gy * width + gx
                occ = data[idx]

                # Only plan to known free cells.
                if occ < 0 or occ > self.free_threshold:
                    continue

                x = ox + (gx + 0.5) * res
                y = oy + (gy + 0.5) * res
                if not self._in_bounds(x, y):
                    continue

                goals.append(GridGoal(gx=gx, gy=gy, x=x, y=y))

        return goals


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WifiAreaSurveyor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
