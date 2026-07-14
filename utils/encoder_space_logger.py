import serial
import time
import csv
import sys
import select
import termios
import tty
import re

PORT = "/dev/ttyACM0"
BAUD = 57600
CSV_FILE = "encoder_mark_log.csv"
ENC_LINE_RE = re.compile(r"^ENC,(\d+),(-?\d+),(-?\d+)\b")
MARK_LINE_RE = re.compile(r"^MARK,(START|SPACE),(\d+),(-?\d+)\b")

def fmt_value(value):
    return "-" if value is None else str(value)

def render_status(latest_arduino_ms, latest_count, latest_dcount,
                  start_arduino_ms, start_count, last_saved, status):
    if latest_count is not None and start_count is not None:
        live_pulse = latest_count - start_count
        live_elapsed = latest_arduino_ms - start_arduino_ms
        live_calc = f"{latest_count} - {start_count} = {live_pulse}"
    else:
        live_pulse = None
        live_elapsed = None
        live_calc = "-"

    lines = [
        "=== Encoder Space Logger ===",
        "keys: s=start, space=save pulse, r=reset Arduino, q=quit",
        "",
        f"current_time_ms : {fmt_value(latest_arduino_ms)}",
        f"current_count   : {fmt_value(latest_count)}",
        f"current_dcount  : {fmt_value(latest_dcount)}",
        "",
        f"start_time_ms   : {fmt_value(start_arduino_ms)}",
        f"start_count     : {fmt_value(start_count)}",
        f"live_pulse      : {live_calc}",
        f"live_elapsed_ms : {fmt_value(live_elapsed)}",
        "",
        f"saved_pulse     : {last_saved.get('pulse', '-')}",
        f"saved_calc      : {last_saved.get('calc', '-')}",
        f"saved_elapsed   : {last_saved.get('elapsed', '-')}",
        f"saved_at_count  : {last_saved.get('count', '-')}",
        "",
        f"status          : {status}",
    ]
    sys.stdout.write("\033[H\033[J" + "\n".join(lines) + "\n")
    sys.stdout.flush()

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.01)
    time.sleep(2)
    ser.reset_input_buffer()

    latest_arduino_ms = None
    latest_count = None
    latest_dcount = None
    start_arduino_ms = None
    start_count = None
    waiting_for_start = False
    waiting_for_space = False
    last_saved = {}
    status = "waiting for encoder data"
    last_render = 0.0

    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "event",
            "pc_time_sec",
            "arduino_time_ms",
            "encoder_count",
            "dcount",
            "start_arduino_ms",
            "start_count",
            "elapsed_ms_from_start",
            "pulse_from_start"
        ])

        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin.fileno())
            render_status(latest_arduino_ms, latest_count, latest_dcount,
                          start_arduino_ms, start_count, last_saved, status)

            while True:
                line = ser.readline().decode(errors="ignore").strip()

                if line.startswith("RESET"):
                    latest_arduino_ms = 0
                    latest_count = 0
                    latest_dcount = 0
                    start_arduino_ms = None
                    start_count = None
                    waiting_for_start = False
                    waiting_for_space = False
                    last_saved = {}
                    writer.writerow([
                        "RESET",
                        time.time(),
                        latest_arduino_ms,
                        latest_count,
                        latest_dcount,
                        "",
                        "",
                        "",
                        ""
                    ])
                    f.flush()
                    status = "RESET acknowledged: start point cleared"
                    render_status(latest_arduino_ms, latest_count, latest_dcount,
                                  start_arduino_ms, start_count, last_saved, status)
                    continue

                mark = MARK_LINE_RE.match(line)
                if mark:
                    mark_type = mark.group(1)
                    mark_arduino_ms = int(mark.group(2))
                    mark_count = int(mark.group(3))

                    latest_arduino_ms = mark_arduino_ms
                    latest_count = mark_count

                    if mark_type == "START":
                        start_arduino_ms = mark_arduino_ms
                        start_count = mark_count
                        waiting_for_start = False
                        status = "START SET"

                        writer.writerow([
                            "START",
                            time.time(),
                            mark_arduino_ms,
                            mark_count,
                            "",
                            start_arduino_ms,
                            start_count,
                            0,
                            0
                        ])
                        f.flush()
                        render_status(latest_arduino_ms, latest_count, latest_dcount,
                                      start_arduino_ms, start_count, last_saved, status)
                    elif mark_type == "SPACE":
                        waiting_for_space = False

                        if start_count is not None:
                            elapsed_ms = mark_arduino_ms - start_arduino_ms
                            pulse = mark_count - start_count
                            last_saved = {
                                "pulse": pulse,
                                "calc": f"{mark_count} - {start_count} = {pulse}",
                                "elapsed": f"{mark_arduino_ms} - {start_arduino_ms} = {elapsed_ms} ms",
                                "count": mark_count,
                            }
                            status = "SPACE saved"

                            writer.writerow([
                                "SPACE",
                                time.time(),
                                mark_arduino_ms,
                                mark_count,
                                "",
                                start_arduino_ms,
                                start_count,
                                elapsed_ms,
                                pulse
                            ])
                            f.flush()
                            render_status(latest_arduino_ms, latest_count, latest_dcount,
                                          start_arduino_ms, start_count, last_saved, status)
                        else:
                            status = "SPACE mark received, but start point is not set"
                            render_status(latest_arduino_ms, latest_count, latest_dcount,
                                          start_arduino_ms, start_count, last_saved, status)

                    continue

                match = ENC_LINE_RE.match(line)
                if match:
                    latest_arduino_ms = int(match.group(1))
                    latest_count = int(match.group(2))
                    latest_dcount = int(match.group(3))

                    now = time.time()
                    if now - last_render >= 0.1:
                        render_status(latest_arduino_ms, latest_count, latest_dcount,
                                      start_arduino_ms, start_count, last_saved, status)
                        last_render = now

                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)

                    if key == "r":
                        ser.reset_input_buffer()
                        ser.write(b"r\n")
                        ser.flush()
                        latest_arduino_ms = 0
                        latest_count = 0
                        latest_dcount = 0
                        start_arduino_ms = None
                        start_count = None
                        waiting_for_start = False
                        waiting_for_space = False
                        last_saved = {}
                        status = "RESET command sent"
                        render_status(latest_arduino_ms, latest_count, latest_dcount,
                                      start_arduino_ms, start_count, last_saved, status)

                    elif key == "s":
                        ser.reset_input_buffer()
                        ser.write(b"S\n")
                        ser.flush()
                        waiting_for_start = True
                        status = "START command sent, waiting for Arduino mark"
                        render_status(latest_arduino_ms, latest_count, latest_dcount,
                                      start_arduino_ms, start_count, last_saved, status)

                    elif key == " ":
                        if start_count is None:
                            status = "press s first to set start point"
                            render_status(latest_arduino_ms, latest_count, latest_dcount,
                                          start_arduino_ms, start_count, last_saved, status)
                        elif waiting_for_start:
                            status = "waiting for START response"
                            render_status(latest_arduino_ms, latest_count, latest_dcount,
                                          start_arduino_ms, start_count, last_saved, status)
                        elif waiting_for_space:
                            status = "waiting for previous SPACE response"
                            render_status(latest_arduino_ms, latest_count, latest_dcount,
                                          start_arduino_ms, start_count, last_saved, status)
                        else:
                            ser.reset_input_buffer()
                            ser.write(b"P\n")
                            ser.flush()
                            waiting_for_space = True
                            status = "SPACE command sent, waiting for Arduino mark"
                            render_status(latest_arduino_ms, latest_count, latest_dcount,
                                          start_arduino_ms, start_count, last_saved, status)

                    elif key == "q":
                        sys.stdout.write("\nquit\n")
                        break

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            ser.close()

if __name__ == "__main__":
    main()
