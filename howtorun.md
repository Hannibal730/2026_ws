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
아래 `/odom/encoder*`, `/odom/encoder_imu*` 항목은 개별 테스트용이다.
EKF 기반 주행/시각화만 할 때는 별도로 실행하지 않아도 된다.

## `/odom/encoder*`
휠 엔코더 단독 dead-reckoning
파라미터 수정은 `src/odom_pkg/config/odom_params.yaml`에서 수정한다.
```
ros2 run odom_pkg encoder_odometry --ros-args \
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
EKF는 `/odom/encoder` + `/imu/data`를 융합한다. 이 실행 파일은 `encoder_odometry`를 **같은 프로세스에서 내부 실행**(`publish_tf:=false`)하여 `/odom/encoder`를 직접 발행하고, `ekf_node`를 `/ekf_encoder_imu_node` 이름으로 띄운다. 그래프에는 path 시각화 보조 노드가 `/local_path_viewer_node`로 보이며, 필터 결과를 받아 `/path/local_odometry`를 발행한다. 따라서 별도로 `encoder_odometry`를 실행할 필요는 없고, `/imu/data`(위 `/imu/*` 항목)만 미리 켜져 있으면 된다.
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

EKF odom과 RViz용 로컬 궤적만 확인하려면 아래 터미널 1~3만 실행하면 된다.
MPPI 제어까지 하려면 터미널 4도 추가로 실행한다.

## 2. 터미널 1 — 휠 엔코더 센서 실행

```
ros2 run bridge_pkg serial_bridge --ros-args \
  --params-file src/bridge_pkg/config/bridge_params.yaml
```

## 3. 터미널 2 — IMU 센서 실행

```
ros2 run imu_pkg imu_publisher --ros-args \
  --params-file src/imu_pkg/config/imu_params.yaml
```

## 4. 터미널 3 — `/odom/ekf_encoder_imu` 실행

```
ros2 run odom_pkg ekf_encoder_imu_odometry \
  --params-file src/odom_pkg/config/ekf_encoder_imu_params.yaml \
  --odom-params-file src/odom_pkg/config/odom_params.yaml
```

이 실행 후 핵심 출력은 아래와 같다.

```text
/ekf_encoder_imu_node
  -> /odom/ekf_encoder_imu
  -> /tf

/local_path_viewer_node
  -> /path/local_odometry
```

## 5. 터미널 4 — MPPI controller 실행

```
ros2 launch mppi_bringup_pkg mppi_controller.launch.py
```

`/controller_server`와 `/planner_server`가 모두 `active [3]`이면 RViz goal planning과 MPPI controller가 실행 준비된 상태다.

## 6. 터미널 5 — RViz2 목표 클릭 또는 path 입력

RViz2에서 마우스로 목표점을 줄 때:
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

여기까지는 MPPI가 `/cmd_vel`을 만드는 단계다. 실제 차량을 움직이려면 `/cmd_vel`을 구동/조향 명령으로 변환하는 저수준 제어 노드가 추가로 필요하다.


## 7. 시리얼 브릿지 사용

```
ros2 run bridge_pkg serial_bridge --ros-args \
  --params-file src/bridge_pkg/config/bridge_params.yaml
```

출력 확인
```
ros2 topic echo /auto_throttle
ros2 topic echo /auto_steer_deg
```




# ROS BAG 기록방법
```
ros2 bag record /토픽이름1 /토픽이름2
ros2 bag record -a
```

# gnss 센서 실행 및 RTK 잡는 방법
```
ros2 launch ublox_gps ublox_f9p_launch.py
ros2 launch ublox_gps ublox_f9r_launch.py
ros2 run fix2nmea fix2nmea
ros2 launch ntrip_client ntrip_client_launch.py
```
