import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import String
import serial


EXPECTED_FIELD_COUNT = 9
GRAVITY = 9.80665


def open_serial():
	port = '/dev/tty' + input('EBIMU Port: /dev/tty').strip()
	baudrate = input('Baudrate: ').strip()
	try:
		return serial.Serial(port=port, baudrate=baudrate, timeout=0.002)
	except serial.SerialException:
		print('Serial port error!')
		raise


ser = open_serial()


def configure_ebimu_runtime_only():
	commands = [
		'<start>',
		'<soc1>',
		'<sof1>',
		'<sog1>',
		'<soa1>',
		'<som0>',
		'<sod0>',
		'<sor10>',
	]
	for command in commands:
		ser.write(command.encode('ascii'))
		ser.flush()
		time.sleep(0.08)
	print('EBIMU runtime config sent: ASCII + Euler + gyro + accel, 100 Hz')


class EbimuPublisher(Node):

	def __init__(self):
		super().__init__('ebimu_publisher')
		qos_profile = QoSProfile(depth=10)

		self.declare_parameter('accel_x_sign', 1.0)
		self.declare_parameter('accel_y_sign', -1.0)
		self.declare_parameter('accel_z_sign', 1.0)
		self.declare_parameter('gyro_x_sign', 1.0)
		self.declare_parameter('gyro_y_sign', -1.0)
		self.declare_parameter('gyro_z_sign', -1.0)
		self.declare_parameter('roll_sign', 1.0)
		self.declare_parameter('pitch_sign', -1.0)
		self.declare_parameter('yaw_sign', -1.0)
		self.declare_parameter('expected_field_count', EXPECTED_FIELD_COUNT)
		self.declare_parameter('allow_extra_fields_after_accel', True)
		self.declare_parameter('serial_poll_period_sec', 0.01)
		self.declare_parameter('imu_publish_rate_hz', 100.0)
		self.declare_parameter('debug_publish_rate_hz', 10.0)
		self.declare_parameter('gyro_integration_dt_sec', 0.01)
		self.declare_parameter('use_integrated_gyro_orientation', True)

		self.accel_x_sign = float(self.get_parameter('accel_x_sign').value)
		self.accel_y_sign = float(self.get_parameter('accel_y_sign').value)
		self.accel_z_sign = float(self.get_parameter('accel_z_sign').value)
		self.gyro_x_sign = float(self.get_parameter('gyro_x_sign').value)
		self.gyro_y_sign = float(self.get_parameter('gyro_y_sign').value)
		self.gyro_z_sign = float(self.get_parameter('gyro_z_sign').value)
		self.roll_sign = float(self.get_parameter('roll_sign').value)
		self.pitch_sign = float(self.get_parameter('pitch_sign').value)
		self.yaw_sign = float(self.get_parameter('yaw_sign').value)
		self.expected_field_count = int(self.get_parameter('expected_field_count').value)
		self.allow_extra_fields_after_accel = bool(
			self.get_parameter('allow_extra_fields_after_accel').value
		)
		self.serial_poll_period_sec = float(
			self.get_parameter('serial_poll_period_sec').value
		)
		self.imu_publish_period = self.period_from_rate(
			float(self.get_parameter('imu_publish_rate_hz').value)
		)
		self.debug_publish_period = self.period_from_rate(
			float(self.get_parameter('debug_publish_rate_hz').value)
		)
		self.gyro_integration_dt_sec = float(
			self.get_parameter('gyro_integration_dt_sec').value
		)
		self.use_integrated_gyro_orientation = bool(
			self.get_parameter('use_integrated_gyro_orientation').value
		)

		self.publisher = self.create_publisher(String, 'ebimu_data', qos_profile)
		self.roll_publisher = self.create_publisher(Float64, 'imu/roll', qos_profile)
		self.pitch_publisher = self.create_publisher(Float64, 'imu/pitch', qos_profile)
		self.yaw_publisher = self.create_publisher(Float64, 'imu/yaw', qos_profile)
		self.gyro_angle_x_publisher = self.create_publisher(Float64, 'imu/gyro_angle/x', qos_profile)
		self.gyro_angle_y_publisher = self.create_publisher(Float64, 'imu/gyro_angle/y', qos_profile)
		self.gyro_angle_z_publisher = self.create_publisher(Float64, 'imu/gyro_angle/z', qos_profile)
		self.gyro_angle_roll_publisher = self.create_publisher(Float64, 'imu/gyro_angle_deg/roll', qos_profile)
		self.gyro_angle_pitch_publisher = self.create_publisher(Float64, 'imu/gyro_angle_deg/pitch', qos_profile)
		self.gyro_angle_yaw_publisher = self.create_publisher(Float64, 'imu/gyro_angle_deg/yaw', qos_profile)
		self.ax_publisher = self.create_publisher(Float64, 'imu/accel/x', qos_profile)
		self.ay_publisher = self.create_publisher(Float64, 'imu/accel/y', qos_profile)
		self.az_publisher = self.create_publisher(Float64, 'imu/accel/z', qos_profile)
		self.gx_publisher = self.create_publisher(Float64, 'imu/gyro/x', qos_profile)
		self.gy_publisher = self.create_publisher(Float64, 'imu/gyro/y', qos_profile)
		self.gz_publisher = self.create_publisher(Float64, 'imu/gyro/z', qos_profile)
		self.imu_publisher = self.create_publisher(Imu, 'imu/data', qos_profile)

		self.invalid_sample_count = 0
		self.empty_read_count = 0
		self.first_packet_logged = False
		self.last_imu_publish_time = 0.0
		self.last_debug_publish_time = 0.0
		self.last_log_times = {}
		self.gyro_angle = [0.0, 0.0, 0.0]
		self.timer = self.create_timer(self.serial_poll_period_sec, self.timer_callback)

	def timer_callback(self):
		try:
			raw_bytes = ser.readline()
		except serial.SerialException as exc:
			self.log_throttled(
				'serial_read',
				f'EBIMU serial read error: {exc}. '
				'Check that the port is connected and not opened by another node.',
				'warn',
			)
			return

		raw = raw_bytes.decode('ascii', errors='ignore').strip()
		if not raw:
			self.empty_read_count += 1
			if self.empty_read_count % 300 == 0:
				self.get_logger().warn(
					'No serial data. Check EBIMU power, port, baudrate, and output mode.'
				)
			return

		if '*' in raw and not raw.startswith('*'):
			raw = raw[raw.index('*'):]
		if not raw.startswith('*'):
			self.log_throttled('nondat', f'Ignoring non-data serial line: {raw}')
			return

		try:
			values = self.parse_values(raw)
		except ValueError:
			self.invalid_sample_count += 1
			self.log_bad_packet(raw, 0)
			return

		if len(values) < self.expected_field_count:
			self.invalid_sample_count += 1
			self.log_bad_packet(raw, len(values))
			return
		if len(values) > self.expected_field_count:
			if not self.allow_extra_fields_after_accel:
				self.invalid_sample_count += 1
				self.log_bad_packet(raw, len(values))
				return
			self.log_throttled(
				'extra_fields',
				f'EBIMU packet has {len(values)} fields; using first '
				f'{self.expected_field_count} and ignoring trailing fields.',
			)
			values = values[:self.expected_field_count]

		roll_deg = values[0] * self.roll_sign
		pitch_deg = values[1] * self.pitch_sign
		yaw_deg = values[2] * self.yaw_sign
		gx = math.radians(values[3] * self.gyro_x_sign)
		gy = math.radians(values[4] * self.gyro_y_sign)
		gz = math.radians(values[5] * self.gyro_z_sign)
		ax = values[6] * self.accel_x_sign * GRAVITY
		ay = values[7] * self.accel_y_sign * GRAVITY
		az = values[8] * self.accel_z_sign * GRAVITY

		if not self.all_finite([roll_deg, pitch_deg, yaw_deg, gx, gy, gz, ax, ay, az]):
			self.invalid_sample_count += 1
			self.log_throttled('finite', 'Discarding IMU sample with NaN/Inf.', 'warn')
			return

		self.integrate_gyro(gx, gy, gz)

		if not self.first_packet_logged:
			self.first_packet_logged = True
			self.get_logger().info(f'First valid EBIMU packet: {raw}')
			self.get_logger().info(
				'Expected fields: roll,pitch,yaw,gx,gy,gz,ax,ay,az; '
				'accel unit from EBIMU manual is g, converted to m/s^2'
			)

		now_sec = self.get_clock().now().nanoseconds * 1e-9
		if self.should_publish(
			now_sec, self.last_debug_publish_time, self.debug_publish_period
		):
			raw_msg = String()
			raw_msg.data = raw
			self.publisher.publish(raw_msg)
			self.publish_float(self.roll_publisher, roll_deg)
			self.publish_float(self.pitch_publisher, pitch_deg)
			self.publish_float(self.yaw_publisher, yaw_deg)
			self.publish_float(self.gx_publisher, values[3] * self.gyro_x_sign)
			self.publish_float(self.gy_publisher, values[4] * self.gyro_y_sign)
			self.publish_float(self.gz_publisher, values[5] * self.gyro_z_sign)
			self.publish_float(self.ax_publisher, ax)
			self.publish_float(self.ay_publisher, ay)
			self.publish_float(self.az_publisher, az)
			self.last_debug_publish_time = now_sec

		if self.should_publish(now_sec, self.last_imu_publish_time, self.imu_publish_period):
			self.publish_imu(roll_deg, pitch_deg, yaw_deg, gx, gy, gz, ax, ay, az)
			self.publish_gyro_angles()
			self.last_imu_publish_time = now_sec

	def publish_imu(self, roll_deg, pitch_deg, yaw_deg, gx, gy, gz, ax, ay, az):
		msg = Imu()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'imu_link'
		if self.use_integrated_gyro_orientation:
			msg.orientation = self.euler_to_quaternion(
				self.gyro_angle[0],
				self.gyro_angle[1],
				self.gyro_angle[2],
			)
		else:
			msg.orientation = self.euler_to_quaternion(
				math.radians(roll_deg),
				math.radians(pitch_deg),
				math.radians(yaw_deg),
			)
		msg.angular_velocity.x = gx
		msg.angular_velocity.y = gy
		msg.angular_velocity.z = gz
		msg.linear_acceleration.x = ax
		msg.linear_acceleration.y = ay
		msg.linear_acceleration.z = az

		msg.orientation_covariance = [
			0.05, 0.0, 0.0,
			0.0, 0.05, 0.0,
			0.0, 0.0, 0.10,
		]
		msg.angular_velocity_covariance = [
			0.0025, 0.0, 0.0,
			0.0, 0.0025, 0.0,
			0.0, 0.0, 0.0025,
		]
		msg.linear_acceleration_covariance = [
			0.25, 0.0, 0.0,
			0.0, 0.25, 0.0,
			0.0, 0.0, 0.25,
		]
		self.imu_publisher.publish(msg)

	def integrate_gyro(self, gx, gy, gz):
		dt = self.gyro_integration_dt_sec
		if dt <= 0.0:
			dt = 0.01
		self.gyro_angle[0] = self.normalize_angle(self.gyro_angle[0] + gx * dt)
		self.gyro_angle[1] = self.normalize_angle(self.gyro_angle[1] + gy * dt)
		self.gyro_angle[2] = self.normalize_angle(self.gyro_angle[2] + gz * dt)

	def publish_gyro_angles(self):
		self.publish_float(self.gyro_angle_x_publisher, self.gyro_angle[0])
		self.publish_float(self.gyro_angle_y_publisher, self.gyro_angle[1])
		self.publish_float(self.gyro_angle_z_publisher, self.gyro_angle[2])
		self.publish_float(self.gyro_angle_roll_publisher, math.degrees(self.gyro_angle[0]))
		self.publish_float(self.gyro_angle_pitch_publisher, math.degrees(self.gyro_angle[1]))
		self.publish_float(self.gyro_angle_yaw_publisher, math.degrees(self.gyro_angle[2]))

	def publish_float(self, publisher, value):
		msg = Float64()
		msg.data = value
		publisher.publish(msg)

	def period_from_rate(self, rate_hz):
		if rate_hz <= 0.0:
			return 0.0
		return 1.0 / rate_hz

	def should_publish(self, stamp_time, last_time, period):
		if period <= 0.0 or last_time <= 0.0:
			return True
		return stamp_time - last_time >= period

	def parse_values(self, raw):
		words = raw.strip().replace('*', '', 1).split(',')
		if not words:
			raise ValueError
		return [float(word) for word in words if word != '']

	def log_bad_packet(self, raw, field_count):
		self.log_throttled(
			'bad_packet',
			'Discarding EBIMU packet: '
			f'field_count={field_count}, expected={self.expected_field_count}, '
			f'raw="{raw}", mode=ASCII Euler+gyro+accel',
			'warn',
		)

	def log_throttled(self, key, message, level='info', period=2.0):
		now = self.get_clock().now().nanoseconds * 1e-9
		last = self.last_log_times.get(key)
		if last is not None and now - last < period:
			return
		self.last_log_times[key] = now
		if level == 'warn':
			self.get_logger().warn(message)
		else:
			self.get_logger().info(message)

	def all_finite(self, values):
		return all(math.isfinite(value) for value in values)

	def normalize_angle(self, angle):
		return math.atan2(math.sin(angle), math.cos(angle))

	def euler_to_quaternion(self, roll, pitch, yaw):
		cy = math.cos(yaw * 0.5)
		sy = math.sin(yaw * 0.5)
		cp = math.cos(pitch * 0.5)
		sp = math.sin(pitch * 0.5)
		cr = math.cos(roll * 0.5)
		sr = math.sin(roll * 0.5)

		q = Imu().orientation
		q.w = cr * cp * cy + sr * sp * sy
		q.x = sr * cp * cy - cr * sp * sy
		q.y = cr * sp * cy + sr * cp * sy
		q.z = cr * cp * sy - sr * sp * cy
		return q


def main(args=None):
	rclpy.init(args=args)
	print('Starting ebimu_publisher..')
	configure_ebimu_runtime_only()
	node = EbimuPublisher()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
