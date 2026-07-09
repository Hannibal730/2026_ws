import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class FilteredPath(Node):
    def __init__(self):
        super().__init__('filtered_path')

        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('path_topic', '/odometry/filtered/path')
        self.declare_parameter('path_min_distance', 0.02)
        self.declare_parameter('path_max_length', 2000)
        self.declare_parameter('republish_rate_hz', 2.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.path_min_distance = float(self.get_parameter('path_min_distance').value)
        self.path_max_length = int(self.get_parameter('path_max_length').value)
        self.republish_rate_hz = float(self.get_parameter('republish_rate_hz').value)

        self.path = Path()
        self.last_position = None
        self.last_odom_time = None
        self.odom_count = 0
        self.path_publish_count = 0

        path_qos = QoSProfile(depth=1)
        path_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        odom_qos = QoSProfile(depth=10)
        odom_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.path_publisher = self.create_publisher(Path, self.path_topic, path_qos)
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            odom_qos,
        )
        self.republish_timer = self.create_timer(
            self.period_from_rate(self.republish_rate_hz),
            self.republish_path,
        )
        self.status_timer = self.create_timer(2.0, self.log_status)

        self.get_logger().info(
            f'Building path from {self.odom_topic} -> {self.path_topic}'
        )

    def odom_callback(self, msg):
        self.last_odom_time = self.get_clock().now().nanoseconds * 1e-9
        self.odom_count += 1

        if self.odom_count == 1:
            self.get_logger().info(f'First odometry received from {self.odom_topic}')

        position = msg.pose.pose.position
        current_position = [position.x, position.y, position.z]

        if self.last_position is not None:
            distance = math.hypot(
                current_position[0] - self.last_position[0],
                current_position[1] - self.last_position[1],
            )
            if distance < self.path_min_distance:
                self.republish_path()
                return

        self.last_position = current_position

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = msg.header.frame_id
        self.path.poses.append(pose)
        self.path.poses = self.path.poses[-self.path_max_length:]
        self.republish_path()

    def republish_path(self):
        if not self.path.poses:
            return
        self.path_publisher.publish(self.path)
        self.path_publish_count += 1

    def log_status(self):
        if self.last_odom_time is None:
            self.get_logger().warn(
                f'Waiting for odometry on {self.odom_topic}; '
                f'no path will be published until it arrives.'
            )
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - self.last_odom_time
        self.get_logger().info(
            f'filtered_path status: odom_count={self.odom_count}, '
            f'path_poses={len(self.path.poses)}, '
            f'path_publish_count={self.path_publish_count}, '
            f'last_odom_age={age:.2f}s'
        )

    def period_from_rate(self, rate_hz):
        if rate_hz <= 0.0:
            return 1.0
        return 1.0 / rate_hz

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FilteredPath()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
