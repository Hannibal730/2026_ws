import argparse
import math
import os
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from odom_pkg.encoder_odometry import EncoderOdometry


class EkfEncoderImuOdometry(Node):
    def __init__(self, params_file):
        super().__init__('ekf_encoder_imu_odometry')

        self.declare_parameter('odom_topic', '/odom/ekf_encoder_imu')
        self.declare_parameter('path_topic', '/odom/ekf_encoder_imu/path')
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
        self.ekf_return_code = None

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
        self.process_timer = self.create_timer(0.5, self.check_child_processes)

        self.ekf_process = subprocess.Popen(
            [
                'ros2',
                'run',
                'robot_localization',
                'ekf_node',
                '--ros-args',
                '--params-file',
                params_file,
                '-r',
                '__node:=ekf_encoder_imu_filter_node',
                '-r',
                'odometry/filtered:=/odom/ekf_encoder_imu',
            ]
        )

        self.get_logger().info(f'Running EKF with params: {params_file}')
        self.get_logger().info(
            'EKF fuses /odom/encoder + /imu/data. /imu/data must be published externally.'
        )
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
            f'ekf_encoder_imu_odometry status: odom_count={self.odom_count}, '
            f'path_poses={len(self.path.poses)}, '
            f'path_publish_count={self.path_publish_count}, '
            f'last_odom_age={age:.2f}s'
        )

    def check_child_processes(self):
        ekf_return_code = self.ekf_process.poll()
        if ekf_return_code is None:
            return

        self.ekf_return_code = ekf_return_code
        self.get_logger().error(
            f'ekf_encoder_imu_filter_node exited with code {ekf_return_code}'
        )
        rclpy.shutdown()

    def stop_child_processes(self):
        process = self.ekf_process
        if process is None or process.poll() is not None:
            return

        process.send_signal(signal.SIGINT)
        deadline = time.monotonic() + 5.0
        remaining = max(0.0, deadline - time.monotonic())
        try:
            process.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            process.terminate()

        if process.poll() is None:
            process.kill()

    def period_from_rate(self, rate_hz):
        if rate_hz <= 0.0:
            return 1.0
        return 1.0 / rate_hz


def parse_args(argv):
    odom_share = get_package_share_directory('odom_pkg')
    default_params = os.path.join(odom_share, 'config', 'ekf_encoder_imu.yaml')
    default_odom_params = os.path.join(odom_share, 'config', 'odom_params.yaml')

    parser = argparse.ArgumentParser(
        description='Run the encoder+IMU EKF filter and publish its filtered path. '
        'Publishes /odom/encoder in-process (unless --external-encoder); '
        '/imu/data must be published externally.'
    )
    parser.add_argument(
        '--params-file',
        default=default_params,
        help='Parameter file for robot_localization ekf_node.',
    )
    parser.add_argument(
        '--odom-params-file',
        default=default_odom_params,
        help='Parameter file for the in-process encoder_odometry.',
    )
    parser.add_argument(
        '--external-encoder',
        action='store_false',
        dest='run_encoder_odometry',
        help='Do not publish /odom/encoder in-process; '
        'expect it from an externally launched encoder_odometry.',
    )
    return parser.parse_known_args(argv[1:])


def main(args=None):
    argv = sys.argv if args is None else [sys.argv[0], *args]
    parsed_args, ros_args = parse_args(argv)

    rclpy.init(args=[argv[0], *ros_args])
    executor = SingleThreadedExecutor()
    ekf_node = None
    encoder_node = None
    return_code = 0
    try:
        ekf_node = EkfEncoderImuOdometry(parsed_args.params_file)
        executor.add_node(ekf_node)

        if parsed_args.run_encoder_odometry:
            encoder_node = EncoderOdometry(
                cli_args=[
                    '--ros-args',
                    '--params-file',
                    parsed_args.odom_params_file,
                    '-p',
                    'publish_tf:=false',
                ]
            )
            executor.add_node(encoder_node)
            ekf_node.get_logger().info(
                'Publishing /odom/encoder in-process (encoder_odometry integrated).'
            )
        else:
            ekf_node.get_logger().info(
                'Expecting /odom/encoder from an external encoder_odometry.'
            )

        executor.spin()
        if ekf_node.ekf_return_code not in (None, 0):
            return_code = 1
    except KeyboardInterrupt:
        pass
    finally:
        if ekf_node is not None:
            ekf_node.stop_child_processes()
        if encoder_node is not None:
            encoder_node.destroy_node()
        if ekf_node is not None:
            ekf_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(return_code)


if __name__ == '__main__':
    main()
