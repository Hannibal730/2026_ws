import math

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

from odom_pkg.encoder_odometry import EncoderWatchdog


class EncoderImuOdometry(Node):

    def __init__(self):
        super().__init__('encoder_imu_odometry')

        self.declare_parameter('odom_publish_rate_hz', 100.0)
        self.declare_parameter('path_publish_rate_hz', 5.0)
        self.declare_parameter('path_min_distance', 0.02)
        self.declare_parameter('path_max_length', 300)
        self.declare_parameter('max_dt_sec', 0.2)
        self.declare_parameter('encoder_timeout_sec', 0.1)
        self.declare_parameter('sensor_timeout_sec', 0.2)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('imu_offset_x', 0.0)
        self.declare_parameter('heading_topic', 'imu/gyro_angle/z')

        self.odom_publish_period = self.period_from_rate(
            float(self.get_parameter('odom_publish_rate_hz').value)
        )
        self.path_publish_period = self.period_from_rate(
            float(self.get_parameter('path_publish_rate_hz').value)
        )
        self.path_min_distance = float(self.get_parameter('path_min_distance').value)
        self.path_max_length = int(self.get_parameter('path_max_length').value)
        self.max_dt_sec = float(self.get_parameter('max_dt_sec').value)
        self.encoder_timeout_sec = float(
            self.get_parameter('encoder_timeout_sec').value
        )
        self.sensor_timeout_sec = float(self.get_parameter('sensor_timeout_sec').value)
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf_enabled = bool(self.get_parameter('publish_tf').value)
        self.imu_offset_x = float(self.get_parameter('imu_offset_x').value)
        self.heading_topic = self.get_parameter('heading_topic').value

        self.yaw = 0.0
        self.last_yaw = 0.0
        self.yaw_rate = 0.0
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.last_time = None
        self.last_speed = 0.0
        self.last_heading_time = None
        self.last_speed_time = None
        self.encoder_distance = 0.0
        self.last_encoder_distance = None
        self.last_distance_time = None
        self.last_path_position = None
        self.last_path_publish_time = None
        self.last_log_times = {}
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.encoder_watchdog = EncoderWatchdog(self.encoder_timeout_sec)

        self.encoder_imu_odom_publisher = self.create_publisher(Odometry, '/odom/encoder_imu', 10)
        self.path_publisher = self.create_publisher(Path, '/odom/encoder_imu/path', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf_enabled else None
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_sensor_transforms()

        self.heading_subscription = self.create_subscription(
            Float64,
            self.heading_topic,
            self.heading_callback,
            10,
        )
        self.speed_subscription = self.create_subscription(Float64, 'encoder/speed', self.speed_callback, 10)
        self.distance_subscription = self.create_subscription(Float64, 'encoder/distance', self.distance_callback, 10)

        self.path = Path()
        self.path.header.frame_id = self.odom_frame_id
        self.odom_timer = self.create_timer(
            self.odom_publish_period,
            self.timer_callback,
            clock=self.steady_clock,
        )
        self.path_timer = self.create_timer(
            self.path_publish_period if self.path_publish_period > 0.0 else 0.2,
            self.publish_path,
        )
        heading_topic_log = self.heading_topic
        if not heading_topic_log.startswith('/'):
            heading_topic_log = '/' + heading_topic_log
        self.get_logger().info(
            f'Raw encoder+IMU odometry: distance=/encoder/distance, '
            f'speed=/encoder/speed, heading={heading_topic_log}'
        )
        self.get_logger().info(
            f'Encoder watchdog enabled: timeout={self.encoder_timeout_sec:.3f}s, '
            f'output=/odom/encoder_imu'
        )

    def publish_sensor_transforms(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for child_frame, x_offset in (
            ('encoder_link', 0.0),
            ('imu_link', self.imu_offset_x),
        ):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.base_frame_id
            transform.child_frame_id = child_frame
            transform.transform.translation.x = x_offset
            transform.transform.rotation.w = 1.0
            transforms.append(transform)

        self.static_tf_broadcaster.sendTransform(transforms)

    def heading_callback(self, msg):
        stamp_time = self.get_clock().now().nanoseconds * 1e-9
        heading = self.normalize_angle(msg.data)
        dt = None if self.last_heading_time is None else stamp_time - self.last_heading_time

        if dt is not None and dt > 0.0:
            self.yaw_rate = self.angle_diff(heading, self.last_yaw) / dt

        self.last_yaw = heading
        self.yaw = heading
        self.last_heading_time = stamp_time

    def speed_callback(self, msg):
        self.last_speed = msg.data
        self.last_speed_time = self.get_clock().now().nanoseconds * 1e-9
        steady_time = self.steady_clock.now().nanoseconds * 1e-9
        recovered = self.encoder_watchdog.record_sample(steady_time)
        if recovered:
            self.get_logger().info('Encoder speed reception recovered.')

    def distance_callback(self, msg):
        stamp_time = self.get_clock().now().nanoseconds * 1e-9
        distance = msg.data

        if self.last_encoder_distance is not None:
            delta_distance = distance - self.last_encoder_distance
            self.position[0] += math.cos(self.yaw) * delta_distance
            self.position[1] += math.sin(self.yaw) * delta_distance

        self.encoder_distance = distance
        self.last_encoder_distance = distance
        self.last_distance_time = stamp_time

    def timer_callback(self):
        stamp = self.get_clock().now()
        stamp_time = stamp.nanoseconds * 1e-9
        steady_time = self.steady_clock.now().nanoseconds * 1e-9

        if self.last_time is None:
            self.last_time = steady_time
            return

        dt = steady_time - self.last_time
        self.last_time = steady_time
        if dt <= 0.0 or dt > self.max_dt_sec:
            self.log_throttled('dt', f'Ignoring odometry update with invalid dt={dt:.4f}', 'warn')
            return

        speed_fresh, just_timed_out, elapsed_sec = self.encoder_watchdog.check(
            steady_time
        )
        speed = self.last_speed if speed_fresh else 0.0
        if elapsed_sec is None:
            self.log_throttled(
                'encoder_speed',
                'Waiting for encoder_speed samples.',
                'warn',
            )
        elif just_timed_out:
            self.get_logger().warn(
                f'Encoder speed timed out after {elapsed_sec:.3f}s; '
                'publishing zero velocity on /odom/encoder_imu.'
            )

        yaw = self.value_if_fresh(self.yaw, self.last_heading_time, stamp_time, 'imu_heading')
        yaw_rate = self.value_if_fresh(self.yaw_rate, self.last_heading_time, stamp_time, 'imu_heading_rate')
        self.value_if_fresh(self.encoder_distance, self.last_distance_time, stamp_time, 'encoder_distance')

        self.yaw = self.normalize_angle(yaw)
        self.velocity[0] = speed
        self.velocity[1] = 0.0
        self.velocity[2] = 0.0

        q = self.yaw_to_quaternion(self.yaw)
        self.publish_odometry(stamp, q, yaw_rate)
        if self.publish_tf_enabled:
            self.publish_tf(stamp, q)

        self.update_path(stamp, q)

    def publish_odometry(self, stamp, q, yaw_rate):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.pose.pose.position.x = self.position[0]
        msg.pose.pose.position.y = self.position[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = q
        msg.twist.twist.linear.x = self.velocity[0]
        msg.twist.twist.linear.y = self.velocity[1]
        msg.twist.twist.linear.z = self.velocity[2]
        msg.twist.twist.angular.z = yaw_rate
        self.encoder_imu_odom_publisher.publish(msg)

    def update_path(self, stamp, q):
        if self.last_path_position is not None:
            distance = math.hypot(
                self.position[0] - self.last_path_position[0],
                self.position[1] - self.last_path_position[1],
            )
            if distance < self.path_min_distance:
                return

        self.last_path_position = list(self.position)
        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = self.odom_frame_id
        pose.pose.position.x = self.position[0]
        pose.pose.position.y = self.position[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation = q
        self.path.header.stamp = stamp.to_msg()
        self.path.poses.append(pose)
        self.path.poses = self.path.poses[-self.path_max_length:]

    def publish_path(self):
        if not self.path.poses:
            return
        self.path_publisher.publish(self.path)

    def publish_tf(self, stamp, q):
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.position[0]
        transform.transform.translation.y = self.position[1]
        transform.transform.translation.z = 0.0
        transform.transform.rotation = q
        self.tf_broadcaster.sendTransform(transform)

    def yaw_to_quaternion(self, yaw):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw * 0.5),
            w=math.cos(yaw * 0.5),
        )

    def value_if_fresh(self, value, sample_time, now, name):
        if sample_time is None:
            self.log_throttled(name, f'Waiting for {name} samples.', 'warn')
            return 0.0
        if now - sample_time > self.sensor_timeout_sec:
            self.log_throttled(name, f'{name} sample timed out; using 0.0.', 'warn')
            return 0.0
        return value

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def angle_diff(self, current, previous):
        return self.normalize_angle(current - previous)

    def should_publish(self, stamp_time, last_time, period):
        if period <= 0.0 or last_time is None:
            return True
        return stamp_time - last_time >= period

    def period_from_rate(self, rate_hz):
        if rate_hz <= 0.0:
            return 0.0
        return 1.0 / rate_hz

    def log_throttled(self, key, message, level='info', period=1.0):
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self.last_log_times.get(key)
        if last is not None and now - last < period:
            return
        self.last_log_times[key] = now
        if level == 'warn':
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EncoderImuOdometry()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
