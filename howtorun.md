# clone 하고 초기 설정
```
cd /home/$(whoami)/Mando2026_ws
python3 -m venv .venv
source .venv/bin/activate
pip3 install -r requirements.txt
sudo apt update
sudo apt install -y libasio-dev ros-humble-diagnostic-updater ros-humble-nmea-msgs ros-humble-bondcpp ros-humble-test-msgs behaviortree-cpp-v3 libgraphicsmagick++1-dev libceres-dev libxtensor-dev ros-humble-nanoflann ros-humble-ompl ros-humble-gazebo-ros-pkgs
```

# 기본 준비
```
cd /home/$(whoami)/Mando2026_ws
deactivate 2>/dev/null || true
sv
sr
cbr
si
sudo chmod 666 /dev/tty*
```

# 센서 값 받아오는 노드
## `/encoder/*`, `/cmd_vel` serial bridge
엔코더 읽기와 `/cmd_vel` 쓰기를 같은 MCU 시리얼 포트에서 처리한다.
포트와 baudrate는 `src/bridge_pkg/config/bridge_params.yaml`에서 수정한다.
```
ros2 run bridge_pkg serial_bridge --ros-args \
  --params-file src/bridge_pkg/config/bridge_params.yaml
```

## `/imu/*`
imu센서의 포트와 baudrate는 `src/imu_pkg/config/imu_params.yaml`에서 수정한다.
```
ros2 run imu_pkg imu_publisher --ros-args \
  --params-file src/imu_pkg/config/imu_params.yaml
```

# 오도메트리 생성 노드
## `/odom/encoder*`
휠 엔코더 단독 dead-reckoning
파라미터 수정은 `src/odom_pkg/config/odom_params.yaml`에서 수정한다.
```
ros2 run odom_pkg encoder_odometry --ros-args \
  --params-file src/odom_pkg/config/odom_params.yaml \
  -p publish_tf:=false
```

## `/odom/imu*`
IMU 단독 dead-reckoning (gyro z yaw 적분 + 전진 가속도 적분).
시작 시 센서를 완전히 정지시킨 채 캘리브레이션(기본 5초)이 진행된다.
파라미터 수정은 `src/odom_pkg/config/odom_params.yaml`에서 수정한다.
```
ros2 run odom_pkg imu_odometry --ros-args \
  --params-file src/odom_pkg/config/odom_params.yaml \
  -p publish_tf:=false
```

## `/odom/encoder_imu*`
휠 엔코더와 imu의 dead-reckoning (엔코더+IMU 단순 적분).
파라미터 수정은 `src/odom_pkg/config/odom_params.yaml`에서 수정한다.
```
ros2 run odom_pkg encoder_imu_odometry --ros-args \
  --params-file src/odom_pkg/config/odom_params.yaml \
  -p publish_tf:=true
```

## `/odom/ekf_encoder_imu*`
EKF는 `/odom/encoder` + `/imu/data`를 융합한다. 이 노드는 `encoder_odometry`를 **같은 프로세스에서 내부 실행**(`publish_tf:=false`)하여 `/odom/encoder`를 직접 발행하고, `ekf_node`를 띄운 뒤 필터 결과 path를 만든다. 따라서 별도로 `encoder_odometry`를 실행할 필요는 없고, `/imu/data`(위 `/imu/*` 항목)만 미리 켜져 있으면 된다.
EKF fusion 파라미터는 `src/odom_pkg/config/ekf_encoder_imu_params.yaml`에서 수정하고, 내부 실행되는 encoder odometry 파라미터는 `src/odom_pkg/config/odom_params.yaml`에서 수정한다.
```
ros2 run odom_pkg ekf_encoder_imu_odometry \
  --params-file src/odom_pkg/config/ekf_encoder_imu_params.yaml \
  --odom-params-file src/odom_pkg/config/odom_params.yaml
```

# MPPI 실행

## 1. `/odom/ekf_encoder_imu` 입력

MPPI는 `src/mppi_bringup_pkg/config/mppi_controller.yaml`에서 아래처럼 `/odom/ekf_encoder_imu`를 사용한다.

```yaml
controller_server:
  ros__parameters:
    odom_topic: /odom/ekf_encoder_imu
```

## 2. 터미널 1 — 휠 엔코더 센서 실행

```
cd /home/$(whoami)/Mando2026_ws
sv
sr
si

ros2 run bridge_pkg serial_bridge --ros-args \
  --params-file src/bridge_pkg/config/bridge_params.yaml
```

## 3. 터미널 2 — IMU 센서 실행

```
cd /home/$(whoami)/Mando2026_ws
sv
sr
si

ros2 run imu_pkg imu_publisher --ros-args \
  --params-file src/imu_pkg/config/imu_params.yaml
```

## 4. 터미널 3 — `/odom/ekf_encoder_imu` 실행

```
cd /home/$(whoami)/Mando2026_ws
sv
sr
si

ros2 run odom_pkg ekf_encoder_imu_odometry \
  --params-file src/odom_pkg/config/ekf_encoder_imu_params.yaml \
  --odom-params-file src/odom_pkg/config/odom_params.yaml
```

## 5. 터미널 4 — MPPI controller 실행

```
cd /home/$(whoami)/Mando2026_ws
sv
sr
si

ros2 launch mppi_bringup_pkg mppi_controller.launch.py
```

`/controller_server`와 `/planner_server`가 모두 `active [3]`이면 RViz goal planning과 MPPI controller가 실행 준비된 상태다.

## 6. 터미널 5 — RViz2 목표 클릭 또는 path 입력

RViz2에서 마우스로 목표점을 줄 때:

```
rviz2
```

- Fixed Frame을 `odom`으로 설정
- `2D Goal Pose`로 목표 위치와 방향 클릭
- `mppi_path_client`가 `/goal_pose`를 받아 `planner_server/compute_path_to_pose`로 경로를 생성한 뒤 `FollowPath`로 전송

CSV 또는 별도 노드가 만든 path를 쓸 때:

```
ros2 topic pub --once /mppi/csv_path nav_msgs/msg/Path "{header: {frame_id: odom}, poses: []}"
```

실제 주행용 path는 `poses`에 최소 2개 이상의 `PoseStamped`를 넣어야 한다. CSV 파일을 바로 읽게 하려면:

```
ros2 launch mppi_bringup_pkg mppi_controller.launch.py start_path_client:=false
```

로 MPPI controller만 띄운 뒤, 별도 터미널에서 CSV용 path client를 실행한다.

```
ros2 run mppi_bringup_pkg mppi_path_client --ros-args \
  --params-file src/mppi_bringup_pkg/config/mppi_controller.yaml \
  -p csv_file_path:=/absolute/path/to/path.csv \
  -p auto_send_csv:=true
```

기본 launch(`start_path_client:=true`)를 이미 실행 중이라면 CSV 파일용 `mppi_path_client`를 추가로 띄우지 말고 `/mppi/csv_path` topic 방식으로 path를 넣는다.

## 7. MPPI 출력 확인

```
ros2 topic echo /cmd_vel
ros2 topic echo /mppi/active_path
ros2 topic echo /trajectories
```

주의: 여기까지는 MPPI가 `/cmd_vel`을 만드는 단계다. 실제 차량을 움직이려면 `/cmd_vel`을 구동/조향 명령으로 변환하는 저수준 제어 노드가 추가로 필요하다.


## 8. 시리얼 브릿지 사용

```
ros2 run bridge_pkg serial_bridge --ros-args \
  --params-file src/bridge_pkg/config/bridge_params.yaml
```

출력 확인
```
ros2 topic echo /cmd_vel
ros2 topic echo /auto_steer_deg
```

<!-- ## `/f9p/*`, `/ublox_gps_node/*`
```
ros2 launch ublox_gps ublox_f9p_launch.py serial_port:=/dev/ttyUSB0 baudrate:=115200
```

## `/f9r/*`, `/ublox_gps_node/*`
```
ros2 launch ublox_gps ublox_f9r_launch.py serial_port:=/dev/ttyUSB1 baudrate:=115200 -->



# ROS BAG 만들기
```
ros2 bag record -e "(/imu/.*|/encoder/.*)"
```

---
udev 작업 필요
