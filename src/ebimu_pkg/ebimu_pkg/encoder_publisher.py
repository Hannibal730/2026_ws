import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('poll_period_sec', 0.01)
        self.declare_parameter('meters_per_pulse', 0.0028628686)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.poll_period = float(self.get_parameter('poll_period_sec').value)
        self.meters_per_pulse = float(self.get_parameter('meters_per_pulse').value)

        self.last_elapsed_ms = None
        self.last_count = None

        self.distance_publisher = self.create_publisher(Float64, 'encoder/distance', 10)
        self.speed_publisher = self.create_publisher(Float64, 'encoder/speed', 10)

        self.ser = self.open_serial()
        self.timer = self.create_timer(self.poll_period, self.timer_callback)

    def open_serial(self):
        try:
            self.get_logger().info(f'Opening encoder serial port {self.serial_port} @ {self.baudrate}')
            return serial.Serial(port=self.serial_port, baudrate=self.baudrate, timeout=0.05)
        except serial.SerialException as exc:
            self.get_logger().error(f'Failed to open serial port: {exc}')
            raise

    def timer_callback(self):
        try:
            raw = self.ser.readline().decode('ascii', errors='ignore').strip()
        except serial.SerialException as exc:
            self.get_logger().error(f'Encoder serial read error: {exc}')
            return

        if not raw:
            return

        parts = raw.split(',')
        if len(parts) < 4 or parts[0] != 'ENC':
            return

        try:
            elapsed_ms = float(parts[1])
            total_count = int(float(parts[2]))
            delta_count = int(float(parts[3]))
            sample_dt = float(parts[4]) / 1000.0 if len(parts) >= 5 else None
        except ValueError:
            self.get_logger().warn(f'Invalid encoder line: {raw}')
            return

        if sample_dt is None and self.last_elapsed_ms is not None:
            sample_dt = (elapsed_ms - self.last_elapsed_ms) / 1000.0
        if sample_dt is None or sample_dt <= 0.0:
            dt = self.poll_period
        else:
            dt = sample_dt

        distance_m = total_count * self.meters_per_pulse
        speed_m_s = (delta_count * self.meters_per_pulse) / dt

        self.publish_float(self.distance_publisher, distance_m)
        self.publish_float(self.speed_publisher, speed_m_s)

        self.last_elapsed_ms = elapsed_ms
        self.last_count = total_count

    def publish_float(self, publisher, value):
        msg = Float64()
        msg.data = value
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EncoderPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
