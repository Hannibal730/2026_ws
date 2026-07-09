import serial
import threading
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
        # poll_period는 sample_dt를 못 구했을 때의 fallback dt로만 사용한다.
        self.poll_period = float(self.get_parameter('poll_period_sec').value)
        self.meters_per_pulse = float(self.get_parameter('meters_per_pulse').value)

        self.last_elapsed_ms = None
        self.last_count = None
        self._serial_lock = threading.Lock()

        self.distance_publisher = self.create_publisher(Float64, 'encoder/distance', 10)
        self.speed_publisher = self.create_publisher(Float64, 'encoder/speed', 10)

        self.ser = self.open_serial()

        # 시리얼을 전용 스레드에서 blocking read로 처리한다.
        # 아두이노가 라인을 보내는 즉시 읽어 발행하므로, 타이머 폴링(100Hz)과의
        # 위상 어긋남/백로그 누적 없이 생산 주기(100Hz)와 1:1로 맞는다.
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def open_serial(self):
        try:
            self.get_logger().info(f'Opening encoder serial port {self.serial_port} @ {self.baudrate}')
            # timeout: 라인 대기 중 이 주기로 readline이 깨어 종료 플래그를 확인할 수 있게 함
            return serial.Serial(port=self.serial_port, baudrate=self.baudrate, timeout=0.1)
        except serial.SerialException as exc:
            self.get_logger().error(f'Failed to open serial port: {exc}')
            raise

    def _read_loop(self):
        # 라인이 도착하는 즉시 읽어 처리(event-driven). readline은 '\n'을 만나면 바로
        # 반환하므로 라인이 밀려 있으면 곧바로 다음 반복에서 계속 비워내 백로그가 쌓이지 않는다.
        while self._running:
            try:
                with self._serial_lock:
                    raw = self.ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException as exc:
                self.get_logger().error(f'Encoder serial read error: {exc}')
                return
            if not raw:
                continue  # timeout으로 빈 줄 → 종료 플래그 재확인 후 계속
            self.process_line(raw)

    def process_line(self, raw):
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

    def stop(self):
        stop_line = b'TH 0.000\n'
        try:
            with self._serial_lock:
                self.ser.write(stop_line)
                self.ser.flush()
        except Exception:
            pass
        self._running = False
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EncoderPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()

            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
