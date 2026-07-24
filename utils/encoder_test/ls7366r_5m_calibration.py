#!/usr/bin/env python3

import serial
import time
import select
import sys
import termios
import tty
import csv
from datetime import datetime
from pathlib import Path


PORT = "/dev/ttyACM0"
BAUD = 115200
CALIBRATION_DISTANCE_M = 5.0
OUTPUT_FILE = Path(__file__).resolve().parent / "ls7366r_pulse_measurements.csv"


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    file_exists = OUTPUT_FILE.exists() and OUTPUT_FILE.stat().st_size > 0
    output_file = OUTPUT_FILE.open("a", newline="")
    output_writer = csv.writer(output_file)
    if not file_exists:
        output_writer.writerow(["saved_at", "calculated_pulses", "meters_per_pulse"])
        output_file.flush()

    previous_count = None
    current_count = None
    start_count = None
    last_print_time = 0.0
    old_terminal_settings = termios.tcgetattr(sys.stdin)

    print("s: 기준값 저장 | Space: 기준값과 현재값 차이 계산 | q: 종료")
    print("Arduino 카운터는 초기화하지 않습니다.")

    try:
        tty.setcbreak(sys.stdin.fileno())

        while True:
            ready, _, _ = select.select([ser, sys.stdin], [], [], 0.1)

            for source in ready:
                if source is ser:
                    line = ser.readline().decode(errors="ignore").strip()
                    fields = line.split(",")

                    if len(fields) >= 4 and fields[0] == "ENC":
                        current_count = int(fields[2])
                    elif len(fields) >= 3 and fields[1] == "ENC":
                        current_count = int(fields[2])
                    else:
                        continue
                    delta_from_previous = (
                        0
                        if previous_count is None
                        else current_count - previous_count
                    )
                    previous_count = current_count

                    now = time.monotonic()
                    if now - last_print_time >= 0.2:
                        start_delta = (
                            "-"
                            if start_count is None
                            else str(current_count - start_count)
                        )
                        print(
                            f"\rcurrent={current_count} "
                            f"delta={delta_from_previous} "
                            f"from_start={start_delta}",
                            end="",
                            flush=True,
                        )
                        last_print_time = now
                else:
                    key = sys.stdin.read(1)

                    if key == "s":
                        if current_count is None:
                            print("\n아직 ENC 데이터를 받지 못했습니다.")
                        else:
                            start_count = current_count
                            print("\nSTART 기준 설정")

                    elif key == " ":
                        if start_count is None or current_count is None:
                            print("\ns를 먼저 눌러 기준값을 저장하세요.")
                        else:
                            signed_pulses = current_count - start_count
                            pulse_count = abs(signed_pulses)
                            print(f"\nprevious_count={start_count}")
                            print(f"current_count={current_count}")
                            print(f"calculated_pulses={pulse_count}")

                            if pulse_count > 0:
                                meters_per_pulse = CALIBRATION_DISTANCE_M / pulse_count
                                print(f"meters_per_pulse={meters_per_pulse:.10f}")
                                saved_at = datetime.now().isoformat(timespec="seconds")
                                output_writer.writerow([
                                    saved_at,
                                    pulse_count,
                                    f"{meters_per_pulse:.10f}",
                                ])
                                output_file.flush()
                                print(f"저장 완료: {OUTPUT_FILE}")

                            start_count = None

                    elif key == "q":
                        return

    except KeyboardInterrupt:
        pass

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal_settings)
        output_file.close()
        ser.close()


if __name__ == "__main__":
    main()
