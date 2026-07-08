import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class EncoderFiveMeterDemo(Node):
    def __init__(self):
        super().__init__('encoder_5m_demo')

        self.declare_parameter('distance_topic', 'encoder/distance')
        self.declare_parameter('throttle_cmd_topic', 'encoder/throttle_cmd')
        self.declare_parameter('target_distance_m', 5.0)
        self.declare_parameter('forward_throttle', 0.25)
        self.declare_parameter('command_rate_hz', 20.0)

        self.distance_topic = self.get_parameter('distance_topic').value
        self.throttle_cmd_topic = self.get_parameter('throttle_cmd_topic').value
        self.target_distance_m = float(self.get_parameter('target_distance_m').value)
        self.forward_throttle = self.clamp(
            float(self.get_parameter('forward_throttle').value),
            -1.0,
            1.0,
        )
        self.command_period = self.period_from_rate(
            float(self.get_parameter('command_rate_hz').value)
        )

        self.start_distance = None
        self.current_distance = None
        self.reached_target = False
        self.stop_logged = False

        self.throttle_publisher = self.create_publisher(
            Float64,
            self.throttle_cmd_topic,
            10,
        )
        self.distance_subscription = self.create_subscription(
            Float64,
            self.distance_topic,
            self.distance_callback,
            10,
        )
        self.timer = self.create_timer(self.command_period, self.timer_callback)

        self.get_logger().info(
            f'5m encoder demo armed: distance=/{self.distance_topic}, '
            f'throttle=/{self.throttle_cmd_topic}, '
            f'target={self.target_distance_m:.2f} m, '
            f'forward_throttle={self.forward_throttle:.2f}'
        )

    def distance_callback(self, msg):
        self.current_distance = float(msg.data)
        if self.start_distance is None:
            self.start_distance = self.current_distance
            self.get_logger().info(
                f'Start distance captured: {self.start_distance:.3f} m'
            )

        traveled = self.traveled_distance()
        if traveled >= self.target_distance_m:
            self.reached_target = True

    def timer_callback(self):
        if self.current_distance is None:
            self.publish_throttle(0.0)
            return

        if self.reached_target:
            self.publish_throttle(0.0)
            if not self.stop_logged:
                self.stop_logged = True
                self.get_logger().info(
                    f'Target reached: traveled={self.traveled_distance():.3f} m. '
                    'Publishing stop throttle.'
                )
            return

        self.publish_throttle(self.forward_throttle)

    def traveled_distance(self):
        if self.start_distance is None or self.current_distance is None:
            return 0.0
        return abs(self.current_distance - self.start_distance)

    def publish_throttle(self, value):
        msg = Float64()
        msg.data = self.clamp(value, -1.0, 1.0)
        self.throttle_publisher.publish(msg)

    def period_from_rate(self, rate_hz):
        if rate_hz <= 0.0:
            return 0.05
        return 1.0 / rate_hz

    def clamp(self, value, minimum, maximum):
        return max(minimum, min(maximum, value))


def main(args=None):
    rclpy.init(args=args)
    node = EncoderFiveMeterDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_throttle(0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
