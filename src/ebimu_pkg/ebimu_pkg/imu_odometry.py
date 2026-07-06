import math

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class ImuOdometry(Node):

	def __init__(self):
		super().__init__('imu_odometry')
		self.subscription = self.create_subscription(Imu, 'imu/data', self.callback, 10)
		self.zupt_subscription = self.create_subscription(
			Bool, 'imu/zero_velocity', self.zero_velocity_callback, 10
		)
		self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
		self.path_publisher = self.create_publisher(Path, 'imu/path', 10)
		self.tf_broadcaster = TransformBroadcaster(self)

		self.declare_parameter('calibration_duration_sec', 5.0)
		self.declare_parameter('calibration_min_samples', 200)
		self.declare_parameter('forward_accel_axis', 'y')
		self.declare_parameter('forward_accel_sign', 1.0)
		self.declare_parameter('left_accel_axis', 'x')
		self.declare_parameter('left_accel_sign', -1.0)
		self.declare_parameter('yaw_gyro_axis', 'z')
		self.declare_parameter('yaw_rate_sign', -1.0)
		self.declare_parameter('max_dt_sec', 0.2)
		self.declare_parameter('enable_stationary_guard', True)
		self.declare_parameter('forward_accel_deadband', 0.06)
		self.declare_parameter('motion_start_accel_threshold', 0.12)
		self.declare_parameter('motion_continue_accel_deadband', 0.04)
		self.declare_parameter('motion_hold_sec', 1.0)
		self.declare_parameter('yaw_rate_deadband', 0.035)
		self.declare_parameter('tilt_guard_gyro_threshold', 0.35)
		self.declare_parameter('stationary_hold_sec', 0.9)
		self.declare_parameter('speed_zero_threshold', 0.025)
		self.declare_parameter('speed_decay_rate', 0.8)
		self.declare_parameter('forward_accel_gain', 1.5)
		self.declare_parameter('max_forward_speed', 1.5)
		self.declare_parameter('odom_publish_rate_hz', 30.0)
		self.declare_parameter('path_publish_rate_hz', 5.0)
		self.declare_parameter('path_min_distance', 0.02)
		self.declare_parameter('path_max_length', 300)
		self.declare_parameter('odom_frame_id', 'odom')
		self.declare_parameter('base_frame_id', 'base_link')

		self.calibration_duration_sec = float(
			self.get_parameter('calibration_duration_sec').value
		)
		self.calibration_min_samples = int(
			self.get_parameter('calibration_min_samples').value
		)
		self.forward_accel_axis = self.get_parameter('forward_accel_axis').value
		self.forward_accel_sign = float(self.get_parameter('forward_accel_sign').value)
		self.left_accel_axis = self.get_parameter('left_accel_axis').value
		self.left_accel_sign = float(self.get_parameter('left_accel_sign').value)
		self.yaw_gyro_axis = self.get_parameter('yaw_gyro_axis').value
		self.yaw_rate_sign = float(self.get_parameter('yaw_rate_sign').value)
		self.max_dt_sec = float(self.get_parameter('max_dt_sec').value)
		self.enable_stationary_guard = bool(
			self.get_parameter('enable_stationary_guard').value
		)
		self.forward_accel_deadband = float(
			self.get_parameter('forward_accel_deadband').value
		)
		self.motion_start_accel_threshold = float(
			self.get_parameter('motion_start_accel_threshold').value
		)
		self.motion_continue_accel_deadband = float(
			self.get_parameter('motion_continue_accel_deadband').value
		)
		self.motion_hold_sec = float(self.get_parameter('motion_hold_sec').value)
		self.yaw_rate_deadband = float(self.get_parameter('yaw_rate_deadband').value)
		self.tilt_guard_gyro_threshold = float(
			self.get_parameter('tilt_guard_gyro_threshold').value
		)
		self.stationary_hold_sec = float(
			self.get_parameter('stationary_hold_sec').value
		)
		self.speed_zero_threshold = float(
			self.get_parameter('speed_zero_threshold').value
		)
		self.speed_decay_rate = float(self.get_parameter('speed_decay_rate').value)
		self.forward_accel_gain = float(self.get_parameter('forward_accel_gain').value)
		self.max_forward_speed = float(self.get_parameter('max_forward_speed').value)
		self.odom_publish_period = self.period_from_rate(
			float(self.get_parameter('odom_publish_rate_hz').value)
		)
		self.path_publish_period = self.period_from_rate(
			float(self.get_parameter('path_publish_rate_hz').value)
		)
		self.path_min_distance = float(self.get_parameter('path_min_distance').value)
		self.path_max_length = int(self.get_parameter('path_max_length').value)
		self.odom_frame_id = self.get_parameter('odom_frame_id').value
		self.base_frame_id = self.get_parameter('base_frame_id').value

		self.calibrating = True
		self.calibration_start_time = None
		self.calibration_samples = []
		self.forward_accel_bias = 0.0
		self.left_accel_bias = 0.0
		self.yaw_rate_bias = 0.0
		self.forward_accel_variance = 0.25
		self.yaw_rate_variance = 0.0025

		self.last_time = None
		self.prev_forward_accel = 0.0
		self.prev_yaw_rate = 0.0
		self.linear_quiet_time = 0.0
		self.angular_quiet_time = 0.0
		self.motion_active = False
		self.motion_hold_time = 0.0
		self.yaw = 0.0
		self.forward_speed = 0.0
		self.position = [0.0, 0.0, 0.0]
		self.velocity = [0.0, 0.0, 0.0]
		self.last_path_position = None
		self.last_odom_publish_time = None
		self.last_path_publish_time = None
		self.path = Path()
		self.path.header.frame_id = self.odom_frame_id
		self.last_log_times = {}

	def zero_velocity_callback(self, msg):
		if not msg.data:
			return
		self.forward_speed = 0.0
		self.velocity = [0.0, 0.0, 0.0]
		self.prev_forward_accel = 0.0
		self.prev_yaw_rate = 0.0
		self.linear_quiet_time = 0.0
		self.angular_quiet_time = 0.0
		self.motion_active = False
		self.motion_hold_time = 0.0
		self.get_logger().info('Manual ZUPT received: forward speed reset to zero')

	def callback(self, msg):
		stamp_time = self.stamp_to_seconds(msg.header.stamp)
		forward_accel_raw = self.select_axis(
			msg.linear_acceleration,
			self.forward_accel_axis,
			self.forward_accel_sign,
		)
		left_accel_raw = self.select_axis(
			msg.linear_acceleration,
			self.left_accel_axis,
			self.left_accel_sign,
		)
		yaw_rate_raw = self.select_axis(
			msg.angular_velocity,
			self.yaw_gyro_axis,
			self.yaw_rate_sign,
		)
		tilt_gyro = math.hypot(msg.angular_velocity.x, msg.angular_velocity.y)

		if self.calibrating:
			self.collect_calibration(
				stamp_time, forward_accel_raw, left_accel_raw, yaw_rate_raw
			)
			return

		if self.last_time is None:
			self.last_time = stamp_time
			self.prev_forward_accel = 0.0
			self.prev_yaw_rate = self.apply_deadband(
				yaw_rate_raw - self.yaw_rate_bias,
				self.yaw_rate_deadband,
			)
			return

		dt = stamp_time - self.last_time
		self.last_time = stamp_time
		if dt <= 0.0 or dt > self.max_dt_sec:
			self.log_throttled('dt', f'Discarding IMU sample with invalid dt={dt:.4f}', 'warn')
			return

		forward_accel = self.filter_forward_accel(
			forward_accel_raw - self.forward_accel_bias,
			tilt_gyro,
			dt,
		)
		yaw_rate = self.apply_deadband(
			yaw_rate_raw - self.yaw_rate_bias,
			self.yaw_rate_deadband,
		)

		linear_quiet = (
			self.enable_stationary_guard
			and forward_accel == 0.0
			and not self.motion_active
		)
		angular_quiet = self.enable_stationary_guard and yaw_rate == 0.0
		if linear_quiet:
			self.linear_quiet_time += dt
			self.prev_forward_accel = 0.0
		else:
			self.linear_quiet_time = 0.0
		if angular_quiet:
			self.angular_quiet_time += dt
			self.prev_yaw_rate = 0.0
		else:
			self.angular_quiet_time = 0.0

		yaw_mid = self.yaw + 0.5 * self.prev_yaw_rate * dt
		self.yaw += 0.5 * (self.prev_yaw_rate + yaw_rate) * dt
		forward_speed_prev = self.forward_speed
		self.forward_speed += 0.5 * (self.prev_forward_accel + forward_accel) * dt
		self.forward_speed = self.clamp(
			self.forward_speed,
			-self.max_forward_speed,
			self.max_forward_speed,
		)
		if linear_quiet:
			decay = math.exp(-self.speed_decay_rate * dt)
			self.forward_speed *= decay
			if (
				self.linear_quiet_time >= self.stationary_hold_sec
				or abs(self.forward_speed) < self.speed_zero_threshold
			):
				self.forward_speed = 0.0
				forward_speed_prev = 0.0
		forward_speed_mid = 0.5 * (forward_speed_prev + self.forward_speed)

		world_vx = math.cos(yaw_mid) * forward_speed_mid
		world_vy = math.sin(yaw_mid) * forward_speed_mid
		self.position[0] += world_vx * dt
		self.position[1] += world_vy * dt
		self.position[2] = 0.0
		self.velocity[0] = self.forward_speed
		self.velocity[1] = 0.0
		self.velocity[2] = 0.0

		self.prev_forward_accel = forward_accel
		self.prev_yaw_rate = yaw_rate

		q = self.yaw_to_quaternion(self.yaw)
		if self.should_publish(
			stamp_time, self.last_odom_publish_time, self.odom_publish_period
		):
			self.publish_odometry(msg, q)
			self.publish_tf(msg, q)
			self.last_odom_publish_time = stamp_time
		if self.should_publish(
			stamp_time, self.last_path_publish_time, self.path_publish_period
		):
			self.publish_path(msg, q)
			self.last_path_publish_time = stamp_time

	def collect_calibration(self, stamp_time, forward_accel, left_accel, yaw_rate):
		if self.calibration_start_time is None:
			self.calibration_start_time = stamp_time
			self.get_logger().info(
				f'Calibration started. Keep sensor completely still for '
				f'{self.calibration_duration_sec:.1f} sec.'
			)

		self.calibration_samples.append((forward_accel, left_accel, yaw_rate))
		elapsed = stamp_time - self.calibration_start_time
		self.log_throttled(
			'calibrating',
			f'Calibrating... elapsed={elapsed:.1f}s samples={len(self.calibration_samples)}',
		)

		if elapsed < self.calibration_duration_sec:
			return
		if len(self.calibration_samples) < self.calibration_min_samples:
			return

		forward_values = [sample[0] for sample in self.calibration_samples]
		left_values = [sample[1] for sample in self.calibration_samples]
		yaw_values = [sample[2] for sample in self.calibration_samples]
		self.forward_accel_bias = self.mean(forward_values)
		self.left_accel_bias = self.mean(left_values)
		self.yaw_rate_bias = self.mean(yaw_values)
		self.forward_accel_variance = max(self.variance(forward_values), 0.01)
		self.yaw_rate_variance = max(self.variance(yaw_values), 0.0001)
		self.calibrating = False
		self.last_time = None
		self.prev_forward_accel = 0.0
		self.prev_yaw_rate = 0.0
		self.linear_quiet_time = 0.0
		self.angular_quiet_time = 0.0
		self.motion_active = False
		self.motion_hold_time = 0.0
		self.last_odom_publish_time = None
		self.last_path_publish_time = None
		self.get_logger().info(
			'Calibration complete: '
			f'forward_accel_bias={self.forward_accel_bias:.6f} m/s^2, '
			f'lateral_accel_bias={self.left_accel_bias:.6f} m/s^2, '
			f'yaw_rate_bias={self.yaw_rate_bias:.6f} rad/s'
		)
		self.get_logger().info(
			'Strict IMU-only odometry active: gyro z yaw integration + '
			'forward acceleration integration. Use /imu/zero_velocity for manual ZUPT.'
		)
		self.get_logger().info(
			f'Forward mapping: IMU acceleration {self.forward_accel_axis} axis '
			f'-> {self.base_frame_id} +x red arrow in RViz.'
		)

	def publish_odometry(self, imu_msg, q):
		msg = Odometry()
		msg.header.stamp = imu_msg.header.stamp
		msg.header.frame_id = self.odom_frame_id
		msg.child_frame_id = self.base_frame_id
		msg.pose.pose.position.x = self.position[0]
		msg.pose.pose.position.y = self.position[1]
		msg.pose.pose.position.z = 0.0
		msg.pose.pose.orientation = q
		msg.twist.twist.linear.x = self.velocity[0]
		msg.twist.twist.linear.y = self.velocity[1]
		msg.twist.twist.linear.z = 0.0
		msg.twist.twist.angular.z = self.prev_yaw_rate
		# Keep published covariance display-friendly. Large z/roll/pitch values
		# make RViz draw a huge covariance disk that hides the path.
		position_var = 0.05
		yaw_var = max(0.05, min(0.2, self.yaw_rate_variance))
		speed_var = max(0.2, min(1.0, self.forward_accel_variance))
		msg.pose.covariance = [
			position_var, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, position_var, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, yaw_var,
		]
		msg.twist.covariance = [
			speed_var, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, speed_var, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, yaw_var,
		]
		self.odom_publisher.publish(msg)

	def publish_path(self, imu_msg, q):
		if self.last_path_position is not None:
			distance = math.hypot(
				self.position[0] - self.last_path_position[0],
				self.position[1] - self.last_path_position[1],
			)
			if distance < self.path_min_distance:
				return
		self.last_path_position = list(self.position)
		pose = PoseStamped()
		pose.header.stamp = imu_msg.header.stamp
		pose.header.frame_id = self.odom_frame_id
		pose.pose.position.x = self.position[0]
		pose.pose.position.y = self.position[1]
		pose.pose.position.z = 0.0
		pose.pose.orientation = q
		self.path.header.stamp = imu_msg.header.stamp
		self.path.poses.append(pose)
		self.path.poses = self.path.poses[-self.path_max_length:]
		self.path_publisher.publish(self.path)

	def publish_tf(self, imu_msg, q):
		transform = TransformStamped()
		transform.header.stamp = imu_msg.header.stamp
		transform.header.frame_id = self.odom_frame_id
		transform.child_frame_id = self.base_frame_id
		transform.transform.translation.x = self.position[0]
		transform.transform.translation.y = self.position[1]
		transform.transform.translation.z = 0.0
		transform.transform.rotation = q
		self.tf_broadcaster.sendTransform(transform)

	def select_axis(self, vector, axis, sign):
		values = {'x': vector.x, 'y': vector.y, 'z': vector.z}
		return values.get(axis, 0.0) * sign

	def filter_forward_accel(self, value, tilt_gyro, dt):
		value *= self.forward_accel_gain
		if tilt_gyro > self.tilt_guard_gyro_threshold:
			self.motion_active = False
			self.motion_hold_time = 0.0
			return 0.0

		abs_value = abs(value)
		if abs_value < self.forward_accel_deadband:
			abs_value = 0.0
			value = 0.0
		if not self.motion_active:
			if abs_value < self.motion_start_accel_threshold:
				return 0.0
			self.motion_active = True
			self.motion_hold_time = self.motion_hold_sec
			return value

		if abs_value >= self.motion_continue_accel_deadband:
			self.motion_hold_time = self.motion_hold_sec
			return value

		self.motion_hold_time = max(0.0, self.motion_hold_time - dt)
		if self.motion_hold_time <= 0.0:
			self.motion_active = False
		return 0.0

	def apply_deadband(self, value, deadband):
		if abs(value) < deadband:
			return 0.0
		return value

	def clamp(self, value, lower, upper):
		return max(lower, min(upper, value))

	def period_from_rate(self, rate_hz):
		if rate_hz <= 0.0:
			return 0.0
		return 1.0 / rate_hz

	def should_publish(self, stamp_time, last_time, period):
		if period <= 0.0 or last_time is None:
			return True
		return stamp_time - last_time >= period

	def yaw_to_quaternion(self, yaw):
		q = Imu().orientation
		q.w = math.cos(yaw * 0.5)
		q.x = 0.0
		q.y = 0.0
		q.z = math.sin(yaw * 0.5)
		return q

	def stamp_to_seconds(self, stamp):
		return stamp.sec + stamp.nanosec * 1e-9

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

	def mean(self, values):
		return sum(values) / float(len(values))

	def variance(self, values):
		if len(values) < 2:
			return 0.0
		mean = self.mean(values)
		return sum((value - mean) ** 2 for value in values) / float(len(values) - 1)


def main(args=None):
	rclpy.init(args=args)
	node = ImuOdometry()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
