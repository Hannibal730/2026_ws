#This is a code for wired IMU sensors.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


def data_parser(msg_data):
	words = msg_data.strip().split(",")    # Fields split

	if(-1 < words[0].find('*')) :
		words[0]=words[0].replace('*','')
		return list(map(float, words)) # float type


def format_imu_data(values):
	if values is None:
		return 'Invalid data'

	if len(values) >= 9:
		return (
			f'Euler  roll: {values[0]:.2f}, pitch: {values[1]:.2f}, yaw: {values[2]:.2f}\n'
			f'Gyro   x: {values[3]:.2f}, y: {values[4]:.2f}, z: {values[5]:.2f}\n'
			f'Accel  x: {values[6]:.3f}, y: {values[7]:.3f}, z: {values[8]:.3f}'
		)

	if len(values) >= 6:
		return (
			f'Accel  x: {values[0]:.3f}, y: {values[1]:.3f}, z: {values[2]:.3f}\n'
			f'Gyro   x: {values[3]:.2f}, y: {values[4]:.2f}, z: {values[5]:.2f}'
		)

	if len(values) >= 3:
		return f'Euler  roll: {values[0]:.2f}, pitch: {values[1]:.2f}, yaw: {values[2]:.2f}'

	return str(values)


class ImuSubscriber(Node):

	def __init__(self):
		super().__init__('imu_subscriber')
		qos_profile = QoSProfile(depth=10)
		self.subscription = self.create_subscription(String, 'imu/raw', self.callback, qos_profile)
		self.subscription   # prevent unuse variable warning

	def callback(self, msg):
		imu_data = data_parser(msg.data)
		print(format_imu_data(imu_data))
		print('---')



def main(args=None):
	rclpy.init(args=args)

	print("Starting imu_subscriber..")

	node = ImuSubscriber()

	try:
		rclpy.spin(node)

	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
