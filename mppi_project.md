# Mando2026 MPPI 적용 계획

목표는 `/home/hannibal/Mando2026_ws/src/navigation2`에 clone된 Nav2 소스를 활용하여 실제 차량에서 `nav2_mppi_controller` 기반 경로 추종을 수행하는 것이다. CARLA에서 성공한 `dual_filter_architecture.md`의 구조를 그대로 가져오되, 실제 차량에서는 CARLA의 `/odometry/local` 역할을 현재 오도메트리인 `/odom/ekf_encoder_imu`가 담당한다.

---

## 1. 핵심 결론

`/odom/ekf_encoder_imu`는 실제 차량 MPPI 제어에서 **local filter 출력**으로 사용한다.

CARLA 문서의 표현으로 바꾸면 다음과 같다.

| CARLA dual filter 용어 | CARLA 토픽 | 실제 차량 토픽 | 역할 |
| :--- | :--- | :--- | :--- |
| local filter output | `/odometry/local` | `/odom/ekf_encoder_imu` | MPPI가 사용할 연속적 local odometry |
| local TF | `odom -> base_link` | `odom -> base_link` | MPPI robot pose 계산용 TF |
| controller odom input | `/odometry/local` | `/odom/ekf_encoder_imu` | MPPI robot_speed 계산용 odom |
| command output | `/cmd_vel` | `/cmd_vel` | MPPI가 출력하는 목표 속도/yaw rate |
| low-level bridge | `cmd_vel_to_carla.py` | `bridge_pkg vehicle_serial_bridge` | `/cmd_vel`을 차량 MCU의 SA/TH 명령으로 변환 |

따라서 Nav2 controller 설정의 핵심은 이것이다.

```yaml
controller_server:
  ros__parameters:
    odom_topic: /odom/ekf_encoder_imu
```

`nav2_mppi_controller`는 `/odom/ekf_encoder_imu`를 직접 구독하는 독립 노드가 아니다. MPPI는 `controller_server` 안에서 plugin으로 실행되며, `controller_server`가 `odom_topic`, TF, FollowPath action, local costmap을 모아 `MPPIController::computeVelocityCommands()`에 전달한다.

```text
/odom/ekf_encoder_imu
  -> controller_server OdomSmoother
  -> robot_speed
  -> MPPI rollout 초기 속도

odom -> base_link TF
  -> controller_server / costmap
  -> robot_pose
  -> MPPI rollout 초기 pose
```

---

## 2. 현재 오도메트리 상태 해석

현재 `/odom/ekf_encoder_imu`는 `nav_msgs/msg/Odometry` 형식이다.

```text
header.frame_id: odom
child_frame_id: base_link
pose.pose.position.x/y/z
pose.pose.orientation
twist.twist.linear.x
twist.twist.angular.z
```

MPPI 관점에서 가장 중요한 필드는 `twist.twist`이다.

```text
twist.twist.linear.x   -> 현재 전후진 속도
twist.twist.angular.z  -> 현재 yaw rate
```

그리고 pose는 odometry topic 자체보다 TF 경로가 더 중요하다. Nav2 controller는 보통 TF로 현재 robot pose를 구한다.

```text
odom -> base_link
```

현재 `ekf_encoder_imu.yaml`의 `publish_tf: true`가 유지되어야 하며, 같은 TF를 다른 노드가 중복 발행하면 안 된다.

---

## 3. 전체 시스템 구조

초기 목표 구조는 다음과 같다.

```text
encoder/speed + encoder/distance
        ^
        |
bridge_pkg vehicle_serial_bridge
        |
        +-----------------------------+
                                      |
/imu/data                             |
   |                                  |
   v                                  v
odom_pkg ekf_encoder_imu_odometry -> /odom/encoder
        |
        v
robot_localization ekf_node
        |
        +-> /odom/ekf_encoder_imu
        +-> odom -> base_link TF

reference path
        |
        v
FollowPath action
        |
        v
Nav2 controller_server
  - nav2_mppi_controller::MPPIController
  - odom_topic: /odom/ekf_encoder_imu
  - robot_base_frame: base_link
  - global/local frame: odom 또는 map
        |
        v
/cmd_vel
        |
        v
bridge_pkg vehicle_serial_bridge
        |
        v
MCU: SA <deg>, TH <value>
```

CARLA와의 가장 큰 차이는 `/cmd_vel` 이후이다. CARLA에서는 `/cmd_vel`을 CARLA `VehicleControl`로 바꿨지만, 실제 차량에서는 `bridge_pkg`가 `/cmd_vel`을 받아 조향각 `SA`와 추력 `TH` 시리얼 명령으로 바꾼다.

---

## 4. 적용 단계

### Phase 1: local-only MPPI 주행

먼저 GNSS/global filter 없이 `odom` 프레임 안에서 MPPI를 성공시키는 것이 목표다.

이 단계에서는 `/odom/ekf_encoder_imu`를 local filter로 사용하고, reference path도 `odom` 프레임으로 준비한다.

```text
global_frame: odom
robot_base_frame: base_link
odom_topic: /odom/ekf_encoder_imu
path.header.frame_id: odom
```

장점:

- `map -> odom` global TF가 없어도 MPPI 테스트 가능
- GNSS jump와 무관하게 제어 안정성 확인 가능
- `/cmd_vel -> vehicle_serial_bridge -> MCU` 제어 루프를 빠르게 검증 가능

이 단계에서 필요한 것은 “따라갈 경로”다. `/odom/ekf_encoder_imu/path`는 지나온 궤적 시각화용이므로 MPPI 입력 경로로 쓰면 안 된다. 별도의 reference path publisher 또는 FollowPath client가 필요하다.

### Phase 2: dual filter 확장

GNSS 기반 global localization이 준비되면 CARLA 문서의 dual filter 구조로 확장한다.

```text
map -> odom -> base_link
```

이때 `/odom/ekf_encoder_imu`는 계속 local filter 역할을 유지한다.

```text
/odom/ekf_encoder_imu = odom -> base_link용 local filter
global GNSS filter    = map -> odom 보정용 global filter
```

중요한 설계 원칙:

- MPPI의 `odom_topic`은 계속 `/odom/ekf_encoder_imu`를 사용한다.
- GNSS 보정은 `map -> odom`에만 반영한다.
- MPPI가 보는 `odom -> base_link`는 GNSS jump로 순간이동하면 안 된다.

---

## 5. Nav2 설정 패키지 구성 계획

`src/navigation2`는 Nav2 소스 자체이므로 가능하면 직접 수정하지 않는다. 실제 차량용 설정과 launch는 별도 패키지로 두는 것이 좋다.

권장 패키지:

```text
src/mppi_bringup/
  package.xml
  setup.py 또는 CMakeLists.txt
  config/
    nav2_mando_mppi_params.yaml
  launch/
    controller.launch.py
  mppi_bringup/
    follow_path_client.py        # 필요 시
    odom_path_recorder.py        # 필요 시
```

`src/navigation2`는 다음 패키지들을 제공하는 소스 workspace로 사용한다.

```text
nav2_controller
nav2_mppi_controller
nav2_costmap_2d
nav2_lifecycle_manager
nav2_planner       # Phase 2 또는 주차/goal planning에서 사용
```

---

## 6. controller_server 기본 파라미터 초안

초기에는 CARLA의 `nav2_carla_params.yaml`을 실제 차량용으로 복사해서 줄이는 방식이 안전하다.

핵심 변경점:

```text
/odometry/local        -> /odom/ekf_encoder_imu
use_sim_time: true     -> false
CARLA lidar topic      -> 실제 LiDAR topic 또는 임시 costmap 비활성 구성
cmd_vel_to_carla       -> bridge_pkg vehicle_serial_bridge
wheelbase 1.47         -> 실제 차량 wheelbase 0.724 m
min_turning_r          -> 실제 차량 최소 회전반경 실측값
```

초기 YAML 골격:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false

    controller_frequency: 10.0
    odom_topic: /odom/ekf_encoder_imu
    odom_duration: 0.3
    transform_tolerance: 0.5
    costmap_update_timeout: 0.30
    failure_tolerance: 1.5

    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001

    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.3
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      stateful: true
      xy_goal_tolerance: 0.5
      yaw_goal_tolerance: 0.3

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"

      time_steps: 40
      model_dt: 0.1
      batch_size: 800
      iteration_count: 1

      vx_std: 0.2
      wz_std: 0.2
      vx_max: 1.0
      vx_min: -0.2
      wz_max: 0.8

      temperature: 0.3
      gamma: 0.015
      visualize: true
      regenerate_noises: true

      motion_model: "ackermann"
      ackermann:
        plugin: "mppi::AckermannMotionModel"
        min_turning_r: 2.0

      critics:
        [
          "ConstraintCritic",
          "GoalCritic",
          "GoalAngleCritic",
          "PathAlignCritic",
          "PathFollowCritic",
          "PathAngleCritic",
          "PreferForwardCritic"
        ]
```

주의:

- 현재 `/home/hannibal/Mando2026_ws/src/navigation2` 소스 기준으로는 `motion_model: "ackermann"`과 `ackermann:` plugin namespace 이름이 일치해야 한다.
- 만약 다른 Nav2 빌드에서 `Model ackermann is not valid` 같은 오류가 나오면, 해당 빌드의 예제/로그에 맞춰 `motion_model`과 하위 namespace 이름을 같은 문자열로 맞춘다.
- `model_dt`는 `1 / controller_frequency`와 맞춘다. 10 Hz면 `0.1`.
- 첫 실제 차량 테스트에서는 `vx_max`를 낮게 둔다. 예: `0.5 ~ 1.0 m/s`.
- `min_turning_r`는 실제 차량의 최대 조향 상태에서 원 주행으로 측정해 갱신한다.

---

## 7. local_costmap 전략

MPPI는 path 추종만 할 수도 있지만, 장애물 회피까지 하려면 local costmap이 필요하다.

초기 실험은 두 갈래 중 하나를 선택한다.

### 선택 A: 장애물 회피 없이 path tracking만 검증

장점:

- 구성 단순
- `/odom/ekf_encoder_imu -> MPPI -> /cmd_vel -> vehicle_serial_bridge` 루프를 먼저 확인 가능

주의:

- `CostCritic`, obstacle layer 관련 critic은 빼거나 약하게 둔다.
- 안전을 위해 사람이 직접 E-stop 가능한 환경에서 저속으로만 테스트한다.

### 선택 B: LiDAR 기반 local_costmap 포함

실제 LiDAR topic이 준비되면 CARLA의 obstacle layer 구조를 실제 topic으로 치환한다.

예:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 10
      height: 10
      resolution: 0.1
      transform_tolerance: 0.5

      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: true
          clearing: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.8
        cost_scaling_factor: 2.0
```

Phase 1에서는 `global_frame: odom`을 권장한다. Phase 2에서 global path/map을 쓰면 global planner 쪽은 `map`, local controller/costmap은 상황에 따라 `odom` 또는 `map`으로 맞춘다.

---

## 8. FollowPath 입력 경로 준비

MPPI는 path를 직접 만들지 않는다. `controller_server`의 `/follow_path` action에 `nav_msgs/Path`를 넣어줘야 한다.

필요 조건:

```text
path.header.frame_id = odom   # Phase 1
path.poses[*].header.frame_id = odom
path는 지나온 궤적이 아니라 앞으로 따라갈 reference path
waypoint 간격은 가능하면 0.1~0.3 m 수준으로 보간
```

가능한 구현:

1. 수동 주행으로 `/odom/ekf_encoder_imu` pose를 기록한다.
2. 기록한 path를 CSV로 저장한다.
3. CSV를 smoothing/interpolation한다.
4. `follow_path_client`가 현재 위치에서 가장 가까운 waypoint 이후만 잘라 `/follow_path` action으로 전송한다.

CARLA에서 이미 검증된 핵심 로직:

- path 첫 점이 현재 차량 위치와 멀면 MPPI가 바로 실패할 수 있다.
- 따라서 현재 차량 위치 기준 가장 가까운 waypoint를 찾고, 그 이후 경로만 보낸다.
- `/odom/ekf_encoder_imu/path`는 “주행 기록”이므로 reference path로 직접 쓰지 않는다.

---

## 9. /cmd_vel 처리

Nav2 controller_server는 최종적으로 `/cmd_vel`을 발행한다.

```text
cmd_vel.linear.x  = 목표 전후진 속도 [m/s]
cmd_vel.angular.z = 목표 yaw rate [rad/s]
```

실제 차량에서는 `bridge_pkg vehicle_serial_bridge`가 이 값을 받아 MCU 프로토콜로 변환한다.

```text
/cmd_vel
  -> vehicle_serial_bridge
  -> SA <steer_deg>
  -> TH <throttle>
```

현재 `vehicle_serial_bridge.py`의 변환 모델:

```text
steer angle = atan(wheelbase * wz / vx)
throttle    = vx / v_max
```

실차 MPPI와 맞춰야 하는 파라미터:

```text
wheelbase: 0.724
v_max: 실제 안전 최고속 기준
max_steer_deg: MCU/기계 조향 한계
steer_sign: 좌우 부호가 반대면 -1.0
cmd_timeout: /cmd_vel 끊김 시 정지 시간
```

MPPI 테스트 전 반드시 확인할 것:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3}, angular: {z: 0.0}}" -r 5
```

차량이 전진하지 않고 후진하면 encoder 또는 bridge의 부호를 먼저 고친다. MPPI 튜닝은 그 다음이다.

---

## 10. 빌드 절차

Nav2 소스를 사용할 경우 overlay 순서가 중요하다.

```bash
cd /home/hannibal/Mando2026_ws
source /opt/ros/humble/setup.bash

# Nav2 전체 빌드가 무겁다면 우선 필요한 패키지만 선택 빌드한다.
colcon build --symlink-install \
  --packages-select \
  nav2_controller \
  nav2_mppi_controller \
  nav2_costmap_2d \
  nav2_lifecycle_manager \
  nav2_util \
  nav2_core \
  nav2_msgs \
  nav2_ros_common

# 실제 차량 패키지 빌드
colcon build --symlink-install \
  --packages-select bridge_pkg odom_pkg imu_pkg encoder_pkg
```

`mppi_bringup` 패키지를 추가한 뒤에는:

```bash
colcon build --symlink-install --packages-select mppi_bringup
```

빌드 후:

```bash
source /home/hannibal/Mando2026_ws/install/setup.bash
ros2 pkg executables nav2_controller
ros2 pkg executables bridge_pkg
```

---

## 11. 실행 절차

### 터미널 1: 센서/브리지

```bash
source /opt/ros/humble/setup.bash
source /home/hannibal/Mando2026_ws/install/setup.bash
ros2 run bridge_pkg vehicle_serial_bridge
```

확인:

```bash
ros2 topic echo --once /encoder/speed
ros2 topic echo --once /encoder/distance
```

### 터미널 2: IMU

현재 IMU 패키지 실행 방식에 맞춰 `/imu/data`를 발행한다.

확인:

```bash
ros2 topic echo --once /imu/data
```

### 터미널 3: local filter

```bash
source /opt/ros/humble/setup.bash
source /home/hannibal/Mando2026_ws/install/setup.bash
ros2 run odom_pkg ekf_encoder_imu_odometry
```

확인:

```bash
ros2 topic echo --once /odom/ekf_encoder_imu
ros2 run tf2_ros tf2_echo odom base_link
```

### 터미널 4: Nav2 controller_server + lifecycle_manager

`mppi_bringup`이 만들어진 뒤:

```bash
source /opt/ros/humble/setup.bash
source /home/hannibal/Mando2026_ws/install/setup.bash
export OMP_NUM_THREADS=4
ros2 launch mppi_bringup controller.launch.py
```

확인:

```bash
ros2 lifecycle get /controller_server
ros2 action info /follow_path
```

정상 기준:

```text
/controller_server: active [3]
/follow_path: Action servers: 1
```

### 터미널 5: reference path 전송

`follow_path_client` 또는 임시 action client로 `nav_msgs/Path`를 `/follow_path`에 보낸다.

확인:

```bash
ros2 topic echo /cmd_vel
ros2 topic hz /cmd_vel
```

---

## 12. 검증 체크리스트

MPPI를 붙이기 전:

```bash
ros2 topic hz /odom/ekf_encoder_imu
ros2 topic echo --once /odom/ekf_encoder_imu --field twist.twist
ros2 run tf2_ros tf2_echo odom base_link
```

확인할 내용:

- `/odom/ekf_encoder_imu` 주파수가 controller 주파수보다 충분히 높다.
- 정지 상태에서 `twist.twist.linear.x`가 0에 가깝다.
- 전진 시 `linear.x`가 양수인지 확인한다.
- 좌회전/우회전 시 `angular.z` 부호가 ROS 기준과 일치하는지 확인한다.
- bag 종료 또는 encoder 입력 중단 시 `/odom/encoder`와 `/odom/ekf_encoder_imu` 속도가 0으로 떨어진다.
- `odom -> base_link` TF가 하나의 노드에서만 발행된다.

Nav2 실행 후:

```bash
ros2 lifecycle get /controller_server
ros2 action info /follow_path
ros2 topic echo /cmd_vel
ros2 topic echo /trajectories
```

차량 연결 전 dry-run:

- 바퀴를 띄우거나 구동부를 안전 분리한다.
- `/cmd_vel`이 과도한 값을 내지 않는지 확인한다.
- `vehicle_serial_bridge`의 `cmd_timeout`이 동작하는지 확인한다.
- FollowPath 취소 또는 controller 종료 시 TH가 0으로 떨어지는지 확인한다.

---

## 13. 주요 위험 요소와 대응

### 13.1 bag 종료 후 차량이 계속 움직이는 문제

원인:

```text
encoder 입력 중단
-> 마지막 /odom/encoder twist.linear.x 유지
-> EKF predict-only
-> /odom/ekf_encoder_imu가 계속 진행
-> MPPI가 차량이 움직인다고 판단
```

대응:

- `encoder_odometry.py`에 encoder watchdog 추가
- `encoder/speed` timeout 시 `/odom/encoder.twist.twist.linear.x = 0.0` 발행
- position은 마지막 값 유지
- `/odom/ekf_encoder_imu`도 정지 상태로 수렴하는지 확인

### 13.2 속도 부호 문제

MPPI와 ROS 표준에서는 보통:

```text
linear.x > 0: 전진
angular.z > 0: 좌회전(CCW)
```

실제 차량이 전진 중인데 `/odom/ekf_encoder_imu.twist.twist.linear.x < 0`이면 MPPI 전에 encoder/bridge 부호를 고친다.

### 13.3 yaw가 변하지 않는 문제

`orientation.z/w`와 `twist.angular.z`가 계속 0이면 MPPI가 회전 상태를 모른다.

확인:

```bash
ros2 topic echo /imu/data --field angular_velocity
ros2 topic echo /odom/ekf_encoder_imu --field twist.twist.angular
```

### 13.4 TF 중복 발행

`robot_localization ekf_node`가 `odom -> base_link`를 발행한다면, `encoder_odometry`나 다른 odom 노드는 같은 TF를 발행하지 않아야 한다.

현재 `ekf_encoder_imu_odometry.py` 내부의 `EncoderOdometry`는 `publish_tf:=false`로 실행되므로 이 방향은 맞다.

### 13.5 CPU 과부하

MPPI는 CPU를 많이 쓴다.

대응:

- `controller_frequency: 10.0`에서 시작
- `model_dt: 0.1`
- `batch_size: 500~1000`
- `OMP_NUM_THREADS=4`에서 시작
- `/cmd_vel` 주파수가 목표 controller frequency의 80% 이하이면 `batch_size`를 줄인다.

---

## 14. 구현 순서 요약

1. `/odom/ekf_encoder_imu` 정지 안정성 확보
   - encoder watchdog 추가
   - bag 종료/입력 중단 시 속도 0 확인

2. `bridge_pkg` 단독 검증
   - `/cmd_vel` 수동 발행
   - SA/TH 방향, 부호, timeout 확인

3. `mppi_bringup` 패키지 생성
   - `nav2_mando_mppi_params.yaml`
   - `controller.launch.py`
   - 필요 시 `follow_path_client.py`

4. local-only MPPI 구성
   - `odom_topic: /odom/ekf_encoder_imu`
   - `global_frame: odom`
   - `robot_base_frame: base_link`
   - reference path frame도 `odom`

5. 저속 dry-run
   - `vx_max: 0.5~1.0`
   - `batch_size: 500~800`
   - 장애물 critic 최소화 또는 안전 환경에서 costmap 없이 시작

6. 실제 주행 튜닝
   - 속도 상한 증가
   - `min_turning_r` 실측 반영
   - `vx_std`, `wz_std`, `temperature`, critic weight 조정

7. GNSS/global filter 확장
   - `map -> odom` 추가
   - path frame을 `map`으로 확장
   - `/odom/ekf_encoder_imu`는 계속 local filter로 유지

---

## 15. 최종 목표 상태

```text
/odom/ekf_encoder_imu
  = 실제 차량 local filter
  = MPPI odom_topic
  = GNSS jump로부터 분리된 제어용 odometry

Nav2 controller_server
  = FollowPath action server
  = MPPI plugin 실행
  = /cmd_vel 발행

bridge_pkg vehicle_serial_bridge
  = /cmd_vel 수신
  = SA/TH 시리얼 명령 송신
  = encoder/distance, encoder/speed 발행
```

이 구조가 성공하면 CARLA에서 검증된 MPPI 경로 추종 구조를 실제 차량에 옮긴 셈이다. 차이는 local filter 토픽명과 low-level actuator bridge뿐이며, MPPI 입장에서 `/odom/ekf_encoder_imu`는 CARLA의 `/odometry/local`과 같은 자리에 놓인다.
