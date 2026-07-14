#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# serial_bridge
# --------------------------------------------------------------------------
# 차량 MCU와 통신하는 "유일한 시리얼 게이트웨이" 노드.
# 하나의 시리얼 포트를 단독 소유하고 다음을 모두 담당한다(양방향):
#
#   [읽기]  아두이노 → 호스트
#     - "ENC,elapsedMs,count,dCount,dtMs" 라인을 파싱해
#       encoder/distance(Float64), encoder/speed(Float64) 로 발행
#     - 그 외 디버그 문자열(MODE:..., ENCODER_START 등)은 ROS 로그로 표시
#
#   [쓰기]  호스트 → 아두이노
#     - /cmd_vel(geometry_msgs/Twist) 구독
#     - (v, w) → 바이시클 모델로 조향각(deg) + 정규화 추력(-1~1)으로 변환
#     - 아두이노 프로토콜 "SA <deg>\n", "TH <val>\n" 으로 송신
#     - 변환 결과를 /auto_steer_deg(Float32), /auto_throttle(Float32) 로도 발행(모니터링)
#
# 왜 통합인가:
#   아두이노는 단일 UART로 ENC를 내보내며 동시에 SA/TH를 받는다.
#   하나의 시리얼 포트는 두 프로세스가 동시에 못 여니, 읽기(encoder_publisher)와
#   쓰기(serial_bridge)를 한 노드로 합쳐 하나의 핸들 + 락으로 처리해야 한다.
#
# 안전장치:
#   - 시작 후 startup_silence_sec 동안 송신 차단(보드 리셋/초기화 보호)
#   - /cmd_vel 이 cmd_timeout 이상 끊기면 정지(SA 0 / TH 0) 송신
#   - tx_rate_hz 로 마지막 명령을 주기적 재전송 → 아두이노 500ms 타임아웃 회피
#   - 조향각은 ±max_steer_deg 로 클램프(아두이노도 최종 방어로 다시 클램프)

import math
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import Twist
import serial


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # ---------------- 파라미터 ----------------
        # 시리얼
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('startup_silence_sec', 3.0)

        # 엔코더 → 거리/속도
        self.declare_parameter('meters_per_pulse', 0.0028628686)
        self.declare_parameter('poll_period_sec', 0.01)   # sample_dt 못 구할 때 fallback dt

        # cmd_vel → SA/TH 변환
        self.declare_parameter('wheelbase', 0.724)         # ★ 실측 휠베이스[m]
        self.declare_parameter('v_max', 4.44)              # ★ 풀스로틀 대략 최고속[m/s] datasheet 상 16km/h, 4.44m/s
        self.declare_parameter('max_steer_deg', 24.0)     # 아두이노 MAX_STEER_TIRE_DEG 와 일치
        self.declare_parameter('min_speed', 0.20)         # v≈0 특이점 가드[m/s]
        self.declare_parameter('steer_sign', 1.0)         # 좌/우 반대면 -1.0
        self.declare_parameter('tx_rate_hz', 30.0)        # SA/TH 재전송 주기
        self.declare_parameter('cmd_timeout', 0.5)        # cmd_vel 끊김 정지 임계[s]

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        g = lambda n: self.get_parameter(n).value
        self.serial_port = g('serial_port')
        self.baudrate = int(g('baudrate'))
        self.startup_silence_sec = float(g('startup_silence_sec'))

        self.meters_per_pulse = float(g('meters_per_pulse'))
        self.poll_period = float(g('poll_period_sec'))

        self.L = float(g('wheelbase'))
        self.v_max = float(g('v_max'))
        self.max_steer = float(g('max_steer_deg'))
        self.min_speed = float(g('min_speed'))
        self.steer_sign = float(g('steer_sign'))
        self.cmd_timeout = float(g('cmd_timeout'))
        tx_rate_hz = float(g('tx_rate_hz'))

        # ---------------- 상태 ----------------
        self.ser = None
        self._ser_lock = threading.Lock()
        self._stop = False

        self._start_time = time.time()
        self._silence_logged = False

        # 엔코더 파생값 계산용
        self.last_elapsed_ms = None

        # 마지막 제어 명령(재전송/타임아웃용)
        self.last_auto_steer_deg = 0.0
        self.last_auto_throttle = 0.0
        self.last_cmd_t = 0.0

        # ---------------- 발행자 ----------------
        self.distance_pub = self.create_publisher(Float64, 'encoder/distance', 10)
        self.speed_pub = self.create_publisher(Float64, 'encoder/speed', 10)
        self.steer_deg_pub = self.create_publisher(Float32, 'auto_steer_deg', 10)
        self.throttle_pub = self.create_publisher(Float32, 'auto_throttle', 10)

        # ---------------- 구독자 ----------------
        self.create_subscription(Twist, g('cmd_vel_topic'), self.on_cmd_vel, 10)

        # ---------------- 스레드/타이머 ----------------
        # 시리얼 열기/재연결
        self._recon_th = threading.Thread(target=self._reconnect_loop, daemon=True)
        self._recon_th.start()
        # 아두이노 → 호스트 읽기(ENC 파싱 + 디버그 로그)
        self._rx_th = threading.Thread(target=self._reader_loop, daemon=True)
        self._rx_th.start()
        # SA/TH 주기적 송신(타임아웃 방지)은 "전용 스레드"에서 처리한다.
        # 이렇게 하면 시리얼 write(최대 write_timeout 만큼 블로킹 가능)가
        # rclpy executor(콜백 스레드) 밖에서 일어나, on_cmd_vel 등 콜백을 막지 않는다.
        self._tx_period = 1.0 / max(1.0, tx_rate_hz)
        self._tx_th = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_th.start()

        self.get_logger().info(
            f"serial_bridge up: {self.serial_port}@{self.baudrate} | "
            f"L={self.L}m v_max={self.v_max}m/s max_steer={self.max_steer}deg | "
            f"cmd_vel='{g('cmd_vel_topic')}'"
        )

    # ============================================================
    # 시리얼 연결 관리
    # ============================================================
    def _reconnect_loop(self):
        while not self._stop:
            if self.ser is None:
                try:
                    self.get_logger().info(f"Opening serial: {self.serial_port}@{self.baudrate}")
                    s = serial.Serial(port=self.serial_port, baudrate=self.baudrate,
                                      timeout=0.05, write_timeout=0.2)
                    time.sleep(0.3)  # 보드 리셋 안정화
                    with self._ser_lock:
                        self.ser = s
                    self.get_logger().info("Serial connected.")
                except Exception as e:
                    self.get_logger().warn(f"Serial open failed: {e}")
                    time.sleep(1.0)
            time.sleep(0.1)

    # ============================================================
    # 읽기: 아두이노 → 호스트
    # ============================================================
    def _reader_loop(self):
        buf = b""
        while not self._stop:
            with self._ser_lock:
                s = self.ser
            if s is None:
                time.sleep(0.1)
                continue
            try:
                data = s.read(256)
                if not data:
                    continue  # timeout → 종료 플래그 재확인
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    text = line.replace(b'\r', b'').decode('ascii', errors='ignore').strip()
                    if text:
                        self._handle_line(text)
            except Exception as e:
                self.get_logger().warn(f"Serial read failed: {e}")
                self._drop_serial()
                time.sleep(0.2)

    def _handle_line(self, raw: str):
        if raw.startswith('ENC,'):
            self._process_encoder(raw)
        else:
            # 아두이노 디버그/상태 문자열(MODE:..., ENCODER_START, RESET, MARK,... 등)
            self.get_logger().info(f"RX: {raw}")

    def _process_encoder(self, raw: str):
        parts = raw.split(',')
        # 형식: ENC,elapsedMs,count,dCount,dtMs
        if len(parts) < 4:
            return
        try:
            elapsed_ms = float(parts[1])
            total_count = int(float(parts[2]))
            delta_count = int(float(parts[3]))
            sample_dt = float(parts[4]) / 1000.0 if len(parts) >= 5 else None
        except ValueError:
            self.get_logger().warn(f"Invalid encoder line: {raw}")
            return

        if sample_dt is None and self.last_elapsed_ms is not None:
            sample_dt = (elapsed_ms - self.last_elapsed_ms) / 1000.0
        dt = sample_dt if (sample_dt is not None and sample_dt > 0.0) else self.poll_period

        distance_m = total_count * self.meters_per_pulse
        speed_m_s = (delta_count * self.meters_per_pulse) / dt

        self._pub_f64(self.distance_pub, distance_m)
        self._pub_f64(self.speed_pub, speed_m_s)

        self.last_elapsed_ms = elapsed_ms

    # ============================================================
    # 쓰기: 호스트 → 아두이노 (/cmd_vel → SA/TH)
    # ============================================================
    def on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)   # 전진속도 [m/s]
        w = float(msg.angular.z)  # 요레이트 [rad/s]

        # --- 조향각(바이시클 모델): delta = atan(L*w/v), v≈0 특이점 가드 ---
        if abs(v) < self.min_speed:
            v_eff = math.copysign(self.min_speed, v if v != 0.0 else 1.0)
        else:
            v_eff = v
        delta_rad = math.atan(self.L * w / v_eff)
        auto_steer_deg = self.steer_sign * math.degrees(delta_rad)
        auto_steer_deg = max(-self.max_steer, min(self.max_steer, auto_steer_deg))

        # --- 추력(개루프 정규화): TH = v / v_max ---
        auto_throttle = max(-1.0, min(1.0, v / self.v_max if self.v_max > 0.0 else 0.0))

        self.last_auto_steer_deg = auto_steer_deg
        self.last_auto_throttle = auto_throttle
        self.last_cmd_t = time.time()

        # 모니터링용 발행
        self._pub_f32(self.steer_deg_pub, auto_steer_deg)
        self._pub_f32(self.throttle_pub, auto_throttle)

    def _tx_loop(self):
        # 전용 스레드: 마지막 명령을 tx_rate_hz 로 재전송(하트비트).
        # 시리얼 write 를 executor 밖에서 수행해 콜백 블로킹을 방지한다.
        while not self._stop:
            self._send_last_cmd()
            time.sleep(self._tx_period)

    def _send_last_cmd(self):
        # cmd_vel 이 오래 끊기면 안전 정지
        if (time.time() - self.last_cmd_t) > self.cmd_timeout:
            auto_steer_deg, auto_throttle = 0.0, 0.0
        else:
            auto_steer_deg, auto_throttle = self.last_auto_steer_deg, self.last_auto_throttle
        # SA/TH 를 한 번의 write 로 합쳐 락/syscall 을 절반으로 줄인다.
        self._write_line(f"SA {auto_steer_deg:.2f}\nTH {auto_throttle:.3f}\n")

    # ============================================================
    # 저수준 유틸
    # ============================================================
    def _in_startup_silence(self) -> bool:
        elapsed = time.time() - self._start_time
        if elapsed < self.startup_silence_sec:
            if not self._silence_logged:
                self._silence_logged = True
                self.get_logger().info(
                    f"Startup silence... (no serial writes for "
                    f"{self.startup_silence_sec - elapsed:.1f}s more)"
                )
            return True
        return False

    def _write_line(self, line: str) -> bool:
        if self._in_startup_silence():
            return False
        with self._ser_lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(line.encode('ascii'))
                return True
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")
                self._drop_serial(locked=True)
                return False

    def _drop_serial(self, locked: bool = False):
        def _close():
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
        if locked:
            _close()
        else:
            with self._ser_lock:
                _close()

    def _pub_f64(self, pub, value):
        m = Float64()
        m.data = float(value)
        pub.publish(m)

    def _pub_f32(self, pub, value):
        m = Float32()
        m.data = float(value)
        pub.publish(m)

    # ============================================================
    def destroy_node(self):
        self._stop = True
        # 송신 스레드를 먼저 멈춰, 아래의 정지 명령이 "마지막 write"로 확실히 남게 한다.
        try:
            self._tx_th.join(timeout=0.5)
        except Exception:
            pass
        # 마지막으로 정지 명령 시도
        try:
            with self._ser_lock:
                if self.ser is not None:
                    self.ser.write(b"TH 0.000\n")
                    self.ser.flush()
        except Exception:
            pass
        try:
            self._recon_th.join(timeout=0.5)
            self._rx_th.join(timeout=0.5)
        except Exception:
            pass
        self._drop_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
