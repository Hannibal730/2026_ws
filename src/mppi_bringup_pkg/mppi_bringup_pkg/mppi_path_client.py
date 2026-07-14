import csv
import math
import os
from typing import Iterable, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MPPIPathClient(Node):
    """Convert operator goals or prepared paths into Nav2 FollowPath goals."""

    def __init__(self):
        super().__init__('mppi_path_client')

        self.declare_parameter('action_name', 'follow_path')
        self.declare_parameter('planner_action_name', 'compute_path_to_pose')
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'goal_checker')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('path_frame', 'odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('csv_path_topic', '/mppi/csv_path')
        self.declare_parameter('active_path_topic', '/mppi/active_path')
        self.declare_parameter('status_topic', '/mppi/status')
        self.declare_parameter('csv_file_path', '')
        self.declare_parameter('csv_frame', 'odom')
        self.declare_parameter('auto_send_csv', False)

        self.action_name = self.get_parameter('action_name').value
        self.planner_action_name = self.get_parameter('planner_action_name').value
        self.controller_id = self.get_parameter('controller_id').value
        self.goal_checker_id = self.get_parameter('goal_checker_id').value
        self.planner_id = self.get_parameter('planner_id').value
        self.path_frame = self.get_parameter('path_frame').value
        self.csv_file_path = self.get_parameter('csv_file_path').value
        self.csv_frame = self.get_parameter('csv_frame').value
        self.auto_send_csv = bool(self.get_parameter('auto_send_csv').value)

        goal_topic = self.get_parameter('goal_topic').value
        csv_path_topic = self.get_parameter('csv_path_topic').value
        active_path_topic = self.get_parameter('active_path_topic').value
        status_topic = self.get_parameter('status_topic').value

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            self.planner_action_name,
        )
        self.follow_client = ActionClient(self, FollowPath, self.action_name)
        self.current_goal_handle = None
        self.pending_path: Optional[Path] = None
        self.is_planning = False
        self.planning_request_id = 0

        self.active_path_pub = self.create_publisher(Path, active_path_topic, transient_qos)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)
        self.create_subscription(Path, csv_path_topic, self.path_callback, transient_qos)

        self.create_timer(1.0, self.publish_status)

        if self.csv_file_path:
            try:
                path = self.load_csv_path(self.csv_file_path, self.csv_frame)
                self.get_logger().info(
                    f'Loaded CSV path: {self.csv_file_path} ({len(path.poses)} poses)'
                )
                self.active_path_pub.publish(path)
                if self.auto_send_csv:
                    self.send_path(path, source='csv_file')
            except (OSError, ValueError) as exc:
                self.get_logger().error(f'Failed to load CSV path: {exc}')

        self.get_logger().info(
            f'MPPI path client ready: /{self.action_name}, '
            f'planner=/{self.planner_action_name}, '
            f'goal_topic={goal_topic}, csv_path_topic={csv_path_topic}, '
            f'path_frame={self.path_frame}'
        )

    def publish_status(self):
        msg = String()
        if self.is_planning:
            msg.data = 'PLANNING'
        elif self.current_goal_handle is not None:
            msg.data = 'FOLLOWING'
        else:
            msg.data = 'IDLE'
        self.status_pub.publish(msg)

    def goal_callback(self, goal: PoseStamped):
        if goal.header.frame_id and goal.header.frame_id != self.path_frame:
            self.get_logger().error(
                f'Goal frame "{goal.header.frame_id}" != path_frame "{self.path_frame}". '
                'Set RViz fixed frame to odom or add frame transformation support.'
            )
            return

        self.plan_to_goal(goal)

    def plan_to_goal(self, goal: PoseStamped):
        if not self.planner_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f'ComputePathToPose action server "{self.planner_action_name}" is not available.'
            )
            return

        goal_for_planner = PoseStamped()
        goal_for_planner.header = goal.header
        goal_for_planner.pose = goal.pose
        goal_for_planner.header.stamp = self.get_clock().now().to_msg()
        if not goal_for_planner.header.frame_id:
            goal_for_planner.header.frame_id = self.path_frame

        request = ComputePathToPose.Goal()
        request.goal = goal_for_planner
        request.planner_id = self.planner_id
        request.use_start = False

        self.planning_request_id += 1
        request_id = self.planning_request_id
        self.is_planning = True

        self.get_logger().info(
            f'Planning path to RViz goal with planner_id={self.planner_id}, '
            f'frame={goal_for_planner.header.frame_id}'
        )
        send_future = self.planner_client.send_goal_async(request)
        send_future.add_done_callback(
            lambda future: self.planner_goal_response_callback(future, request_id)
        )

    def planner_goal_response_callback(self, future, request_id: int):
        if request_id != self.planning_request_id:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.is_planning = False
            self.get_logger().error(f'ComputePathToPose request failed: {exc}')
            return

        if not goal_handle.accepted:
            self.is_planning = False
            self.get_logger().error('ComputePathToPose goal rejected.')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.planner_result_callback(future, request_id)
        )
        self.get_logger().info('ComputePathToPose goal accepted.')

    def planner_result_callback(self, future, request_id: int):
        if request_id != self.planning_request_id:
            return

        self.is_planning = False
        try:
            result_msg = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'ComputePathToPose result failed: {exc}')
            return

        path = result_msg.result.path
        if not path.poses:
            self.get_logger().error(
                f'Planner returned an empty path. action_status={result_msg.status}'
            )
            return

        if not path.header.frame_id:
            path.header.frame_id = self.path_frame

        self.get_logger().info(
            f'Planner produced path: {len(path.poses)} poses, '
            f'frame={path.header.frame_id}, action_status={result_msg.status}'
        )
        self.send_path(path, source='planner_goal')

    def path_callback(self, path: Path):
        if not path.poses:
            self.get_logger().warn('Ignoring empty path from csv_path_topic.')
            return

        if not path.header.frame_id:
            path.header.frame_id = self.path_frame

        if path.header.frame_id != self.path_frame:
            self.get_logger().error(
                f'Path frame "{path.header.frame_id}" != path_frame "{self.path_frame}". '
                'Publish a path in the controller frame for the first bringup.'
            )
            return

        self.send_path(path, source='path_topic')

    def send_path(self, path: Path, source: str):
        path.header.stamp = self.get_clock().now().to_msg()
        for pose in path.poses:
            pose.header.stamp = path.header.stamp
            if not pose.header.frame_id:
                pose.header.frame_id = path.header.frame_id

        if not self.follow_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f'FollowPath action server "{self.action_name}" is not available.'
            )
            return

        self.active_path_pub.publish(path)
        self.pending_path = path

        if self.current_goal_handle is not None:
            self.get_logger().info('Canceling previous FollowPath goal before sending a new path.')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _: self._send_pending(source))
            return

        self._send_pending(source)

    def _send_pending(self, source: str):
        path = self.pending_path
        self.pending_path = None
        self.current_goal_handle = None
        if path is None:
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = self.controller_id
        goal_msg.goal_checker_id = self.goal_checker_id

        self.get_logger().info(
            f'Sending FollowPath from {source}: {len(path.poses)} poses, '
            f'frame={path.header.frame_id}'
        )
        send_future = self.follow_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected.')
            self.current_goal_handle = None
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
        self.get_logger().info('FollowPath goal accepted.')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'FollowPath feedback speed={feedback.speed:.3f}')

    def result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'FollowPath finished: status={result.status}')
        self.current_goal_handle = None

    def load_csv_path(self, file_path: str, frame_id: str) -> Path:
        expanded_path = os.path.expanduser(file_path)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id

        with open(expanded_path, newline='', encoding='utf-8') as csv_file:
            rows = list(csv.reader(csv_file))

        if not rows:
            raise ValueError(f'{expanded_path} is empty')

        header = None
        first = rows[0]
        if not all(is_float(value) for value in first[:2]):
            header = [value.strip().lower() for value in first]
            rows = rows[1:]

        for row in rows:
            if not row or len(row) < 2:
                continue
            x, y, yaw = parse_csv_pose(row, header)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)

        if len(path.poses) < 2:
            raise ValueError(f'{expanded_path} must contain at least 2 valid poses')

        return path


def parse_csv_pose(row: Iterable[str], header: Optional[list[str]]):
    values = list(row)
    if header:
        value_by_name = {
            name: values[index]
            for index, name in enumerate(header)
            if index < len(values)
        }
        x = float(first_present(value_by_name, ['x', 'odom_x', 'map_x']))
        y = float(first_present(value_by_name, ['y', 'odom_y', 'map_y']))
        yaw_value = first_present(value_by_name, ['yaw', 'theta', 'heading'], default='0.0')
        yaw = float(yaw_value)
    else:
        x = float(values[0])
        y = float(values[1])
        yaw = float(values[2]) if len(values) >= 3 and values[2] else 0.0

    return x, y, yaw


def first_present(values: dict[str, str], names: list[str], default: Optional[str] = None):
    for name in names:
        if name in values and values[name] != '':
            return values[name]
    if default is not None:
        return default
    raise ValueError(f'Missing one of CSV columns: {names}')


def is_float(value: str) -> bool:
    try:
        float(value)
        return True
    except ValueError:
        return False


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def main(args=None):
    rclpy.init(args=args)
    node = MPPIPathClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
