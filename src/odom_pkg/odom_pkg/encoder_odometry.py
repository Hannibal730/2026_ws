import math
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


class EncoderWatchdog:
    """Determine whether encoder speed samples are still fresh."""

    def __init__(self, timeout_sec):
        if timeout_sec <= 0.0:
            raise ValueError('timeout_sec must be greater than 0.0')

        self.timeout_sec = float(timeout_sec)
        self._last_sample_time = None
        self._timeout_active = False
        self._lock = threading.Lock()

    def record_sample(self, sample_time):
        """Record a sample and return True when recovering from timeout."""
        with self._lock:
            recovered = self._timeout_active
            self._last_sample_time = float(sample_time)
            self._timeout_active = False
            return recovered

    def check(self, now):
        """Return (fresh, just_timed_out, elapsed_sec)."""
        with self._lock:
            if self._last_sample_time is None:
                return False, False, None

            elapsed_sec = max(0.0, float(now) - self._last_sample_time)
            if elapsed_sec <= self.timeout_sec:
                return True, False, elapsed_sec

            just_timed_out = not self._timeout_active
            self._timeout_active = True
            return False, just_timed_out, elapsed_sec


class EncoderOdometry(Node):

    def __init__(self, **kwargs):
        super().__init__('encoder_odometry', **kwargs)

        self.declare_parameter('odom_publish_rate_hz', 100.0)
        self.declare_parameter('path_publish_rate_hz', 5.0)
        self.declare_parameter('path_min_distance', 0.02)
        self.declare_parameter('path_max_length', 300)
        self.declare_parameter('max_dt_sec', 0.2)
        self.declare_parameter('encoder_timeout_sec', 0.1)
        self.declare_parameter('odom_topic', '/odom/encoder')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('heading_rad', 0.0)
        self.declare_parameter('publish_tf', True)

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
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.heading_rad = float(self.get_parameter('heading_rad').value)
        self.publish_tf_enabled = bool(self.get_parameter('publish_tf').value)
        if self.odom_publish_period <= 0.0:
            raise ValueError('odom_publish_rate_hz must be greater than 0.0')

        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.last_time = None
        self.last_path_position = None
        self.current_speed = 0.0
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.encoder_watchdog = EncoderWatchdog(self.encoder_timeout_sec)

        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.path_publisher = self.create_publisher(Path, '/odom/encoder/path', 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf_enabled else None
        )

        self.speed_subscription = self.create_subscription(Float64, 'encoder/speed', self.speed_callback, 10)
        self.distance_subscription = self.create_subscription(Float64, 'encoder/distance', self.distance_callback, 10)

        self.path = Path()
        self.path.header.frame_id = self.odom_frame_id
        self.path_timer = self.create_timer(
            self.path_publish_period if self.path_publish_period > 0.0 else 0.2,
            self.publish_path,
        )
        # ROS bag의 /clock이 멈추더라도 timeout을 판정할 수 있도록
        # watchdog timer에는 steady clock을 사용한다.
        self.odom_timer = self.create_timer(
            self.odom_publish_period,
            self.odom_timer_callback,
            clock=self.steady_clock,
        )
        self.get_logger().info(
            f'Encoder watchdog enabled: input=/encoder/speed, '
            f'timeout={self.encoder_timeout_sec:.3f}s, output={self.odom_topic}'
        )

    def speed_callback(self, msg):
        stamp = self.get_clock().now()
        stamp_time = stamp.nanoseconds * 1e-9
        speed = msg.data
        steady_time = self.steady_clock.now().nanoseconds * 1e-9
        recovered = self.encoder_watchdog.record_sample(steady_time)
        if recovered:
            self.get_logger().info('Encoder speed reception recovered.')

        self.current_speed = speed
        self.velocity[0] = speed
        self.velocity[1] = 0.0
        self.velocity[2] = 0.0

        if self.last_time is None:
            self.last_time = stamp_time
            return

        dt = stamp_time - self.last_time
        self.last_time = stamp_time
        if dt <= 0.0 or dt > self.max_dt_sec:
            self.log_throttled('dt', f'Ignoring encoder sample with invalid dt={dt:.4f}', 'warn')
            return

        self.position[0] += math.cos(self.heading_rad) * speed * dt
        self.position[1] += math.sin(self.heading_rad) * speed * dt

        q = self.yaw_to_quaternion(self.heading_rad)
        self.update_path(stamp, q)

    def odom_timer_callback(self):
        stamp = self.get_clock().now()
        steady_time = self.steady_clock.now().nanoseconds * 1e-9
        fresh, just_timed_out, elapsed_sec = self.encoder_watchdog.check(steady_time)

        if fresh:
            self.velocity[0] = self.current_speed
        else:
            self.current_speed = 0.0
            self.velocity[0] = 0.0

        self.velocity[1] = 0.0
        self.velocity[2] = 0.0

        if just_timed_out:
            self.get_logger().warn(
                f'Encoder speed timed out after {elapsed_sec:.3f}s; '
                f'publishing zero velocity on {self.odom_topic}.'
            )

        q = self.yaw_to_quaternion(self.heading_rad)
        self.publish_odometry(stamp, q)
        if self.publish_tf_enabled:
            self.publish_tf(stamp, q)

    def distance_callback(self, msg):
        # distance is available if needed for diagnostics or future use
        self.distance_m = msg.data

    def publish_odometry(self, stamp, q):
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
        msg.twist.twist.angular.z = 0.0
        self.odom_publisher.publish(msg)

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
        q = Imu().orientation
        q.w = math.cos(yaw * 0.5)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        return q

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
        if not hasattr(self, 'last_log_times'):
            self.last_log_times = {}
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
        node = EncoderOdometry()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
