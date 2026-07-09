import argparse
import os
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory


def main():
    odom_share = get_package_share_directory('odom_pkg')
    default_params = os.path.join(odom_share, 'config', 'ekf.yaml')

    parser = argparse.ArgumentParser(
        description='Run the EKF node and its filtered path publisher.'
    )
    parser.add_argument(
        '--params-file',
        default=default_params,
        help='Parameter file for robot_localization ekf_node.',
    )
    args = parser.parse_args()

    commands = [
        (
            'ekf_filter_node',
            [
                'ros2',
                'run',
                'robot_localization',
                'ekf_node',
                '--ros-args',
                '--params-file',
                args.params_file,
                '-r',
                '__node:=ekf_filter_node',
            ],
        ),
        (
            'filtered_path',
            [
                'ros2',
                'run',
                'odom_pkg',
                'filtered_path',
                '--ros-args',
                '-p',
                'odom_topic:=/odometry/filtered',
                '-p',
                'path_topic:=/odometry/filtered/path',
            ],
        ),
    ]

    processes = []
    return_code = 0
    try:
        for name, command in commands:
            processes.append((name, subprocess.Popen(command)))

        while True:
            for name, process in processes:
                process_return_code = process.poll()
                if process_return_code is not None:
                    raise RuntimeError(
                        f'{name} exited with code {process_return_code}'
                    )
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f'[ekf_filter] {exc}', file=sys.stderr, flush=True)
        return_code = 1
    finally:
        stop_processes(processes)

    sys.exit(return_code)


def stop_processes(processes):
    for _, process in reversed(processes):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)

    deadline = time.monotonic() + 5.0
    for _, process in reversed(processes):
        remaining = max(0.0, deadline - time.monotonic())
        if process.poll() is None:
            try:
                process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                process.terminate()

    for _, process in reversed(processes):
        if process.poll() is None:
            process.kill()


if __name__ == '__main__':
    main()
