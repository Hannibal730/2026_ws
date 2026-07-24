# `/odom/ekf_encoder_imu` 기반 Nav2 MPPI 제어 계획서

## 1. 목표

이 문서는 `/home/hannibal/Mando2026_ws/src/navigation2`에 clone된 Nav2 Humble 소스의 `nav2_controller`와 `nav2_mppi_controller`를 사용하여, 현재 차량의 로컬 오도메트리인 `/odom/ekf_encoder_imu`를 입력으로 MPPI 제어를 수행하기 위한 계획, 절차, 설정 기준을 정리한다.

핵심 목표는 다음과 같다.

| 항목 | 목표 |
| :--- | :--- |
| 로컬 상태 추정 | `/odom/ekf_encoder_imu`를 MPPI의 로컬 필터 출력으로 사용 |
| 현재 속도 입력 | `controller_server.odom_topic`에 `/odom/ekf_encoder_imu` 지정 |
| 현재 pose 입력 | TF `odom -> base_link`를 통해 Nav2가 계산 |
| 경로 추종 | `FollowPath` action으로 `nav_msgs/Path`를 MPPI에 전달 |
| 출력 명령 | Nav2 controller server가 `/cmd_vel` 발행 |
| 저수준 제어 | `/cmd_vel`을 실제 차량 구동/조향 명령으로 변환 |

---

## 2. `/odom/ekf_encoder_imu`의 역할

`/odom/ekf_encoder_imu`는 MPPI 관점에서 **로컬 필터(local filter)** 출력이다.

이 토픽은 GNSS나 전역 위치 보정이 섞인 절대 위치가 아니라, 엔코더와 IMU를 기반으로 `odom` 프레임 안에서 차량의 상대 이동을 연속적으로 추정하는 제어용 오도메트리로 사용한다.

현재 예시 메시지 기준:

```text
topic: /odom/ekf_encoder_imu
type:  nav_msgs/Odometry

header.frame_id: odom
child_frame_id:  base_link

pose.pose:
  odom 프레임 기준 base_link 위치/자세

twist.twist:
  linear.x   = 차량 전후방 속도
  angular.z  = yaw rate
```

MPPI에서 중요한 점은 다음 두 가지다.

| MPPI 입력 | 공급 방식 | `/odom/ekf_encoder_imu`의 역할 |
| :--- | :--- | :--- |
| 현재 속도 `robot_speed` | `controller_server.odom_topic` | `twist.twist`를 제공 |
| 현재 자세 `robot_pose` | TF buffer / costmap | `odom -> base_link` TF를 제공 |

즉 `odom_topic`은 주로 `pose.pose`보다 `twist.twist`를 위해 사용된다. 차량의 현재 pose는 Nav2가 TF에서 가져온다.

---

## 3. 전체 아키텍처

1차 목표는 전역 좌표계 없이 `odom` 프레임만으로 MPPI 경로 추종을 성공시키는 것이다.

```text
encoder + IMU
    ↓
odom_pkg / robot_localization EKF
    ↓
/odom/ekf_encoder_imu  +  TF: odom -> base_link
    ↓
Nav2 controller_server
    ├─ odom_topic에서 현재 속도 수신
    ├─ TF에서 현재 pose 계산
    ├─ FollowPath action으로 받은 path를 local plan으로 변환
    ├─ local_costmap으로 장애물 비용 계산
    └─ nav2_mppi_controller::MPPIController 실행
          ↓
        /cmd_vel
          ↓
저수준 차량 제어 노드
```

기본 TF 트리는 다음처럼 단순하게 시작한다.

```text
odom -> base_link
```

전역 지도나 절대 위치 기반 경로를 붙이는 단계에서는 아래 구조로 확장할 수 있다.

```text
map -> odom -> base_link
```

단, 초기 MPPI 검증에서는 `map -> odom`이 없어도 된다. 이 경우 모든 경로와 costmap의 기준 프레임을 `odom`으로 통일한다.

---

## 4. Nav2 MPPI가 요구하는 입력

`nav2_mppi_controller`는 독립 실행 노드가 아니라 `nav2_controller`의 `controller_server` 안에서 로드되는 controller plugin이다. 따라서 MPPI를 쓰려면 단순히 odometry 하나만 연결하는 것이 아니라 다음 조건이 함께 필요하다.

| 요구사항 | 공급 방법 | 본 프로젝트 기준 |
| :--- | :--- | :--- |
| 현재 속도 | `controller_server.odom_topic` | `/odom/ekf_encoder_imu` |
| 현재 pose | TF | `odom -> base_link` |
| 추종 경로 | `FollowPath` action의 `nav_msgs/Path` | 별도 path publisher/client 필요 |
| 최종 goal | path 마지막 pose 또는 action goal | `FollowPath` goal에 포함 |
| costmap | `local_costmap` | 초기에는 rolling local costmap |
| motion model | MPPI parameter | 차량형이면 `Ackermann` 권장 |
| 제약 조건 | MPPI parameter | `vx_max`, `vx_min`, `wz_max`, `min_turning_r` |
| 출력 소비자 | `/cmd_vel` 구독 노드 | 구동/조향 변환 노드 필요 |

주의할 점:

- `/path/local_odometry`는 지나온 궤적을 기록한 시각화용 path이다. MPPI가 따라가야 할 계획 경로로 쓰면 안 된다.
- MPPI에 넣을 경로는 별도의 `nav_msgs/Path`로 생성하고, `FollowPath` action client가 `controller_server/follow_path`로 보내야 한다.

---

## 5. MPPI 운용 모드: 목표점 클릭과 준비된 경로 추종

질문에서 정리한 것처럼 실제 운용 형태는 크게 두 가지로 볼 수 있다.

| 모드 | 사용자 입력 | MPPI에 실제로 들어가는 입력 | 필요한 중간 처리 |
| :--- | :--- | :--- | :--- |
| 목표점 클릭 | RViz2 `2D Goal Pose`의 `/goal_pose` | `nav_msgs/Path` | planner가 현재 위치에서 목표점까지 경로 생성 |
| 준비된 경로 추종 | CSV 또는 이미 만들어둔 `nav_msgs/Path` | `nav_msgs/Path` | CSV 파싱 또는 path topic 수신 |

중요한 점은 **MPPI는 목표점 자체를 직접 따라가는 알고리즘이 아니라 path follower**라는 것이다. 즉 RViz2에서 마우스로 목표 pose를 찍어도, 그 pose가 바로 MPPI로 들어가는 것이 아니다. 별도 노드가 현재 차량 pose와 목표 pose 사이의 경로를 만들거나, Nav2 planner가 경로를 계산한 뒤, 그 결과 path를 `FollowPath` action으로 `controller_server`에 보내야 한다.

```text
[RViz2 2D Goal Pose]
  /goal_pose (PoseStamped)
      ↓
  planner_server/compute_path_to_pose
      ↓
  FollowPath(path)
      ↓
  controller_server + MPPI
      ↓
  /cmd_vel
```

준비된 경로를 쓰는 경우도 최종 입력은 동일하다.

```text
[CSV 또는 사전 생성 Path]
      ↓
  nav_msgs/Path
      ↓
  FollowPath(path)
      ↓
  controller_server + MPPI
      ↓
  /cmd_vel
```

### 5.1 목표점 클릭 모드

현재 구현에서는 `mppi_path_client`가 `/goal_pose`를 구독한 뒤, 목표 pose를 직접 직선 보간하지 않고 `planner_server`의 `ComputePathToPose` action으로 보낸다. RViz2의 Fixed Frame을 `odom`으로 두고 `2D Goal Pose`를 클릭하면 다음 과정을 수행한다.

1. RViz2가 `/goal_pose`에 목표 `PoseStamped`를 발행한다.
2. `mppi_path_client`가 `planner_server/compute_path_to_pose` action을 호출한다.
3. `planner_server`의 `SmacPlannerHybrid`가 `global_costmap`과 최소 회전반경을 고려해 `nav_msgs/Path`를 생성한다.
4. `mppi_path_client`가 planner 결과 path를 `/mppi/active_path`로 발행한다.
5. 같은 path를 `FollowPath` action으로 `controller_server`에 전송한다.
6. `controller_server`의 MPPI plugin이 `/odom/ekf_encoder_imu`와 local costmap을 사용해 `/cmd_vel`을 만든다.

따라서 RViz goal 클릭 모드는 이제 단순 직선 추종이 아니라 아래 구조다.

```text
RViz2 2D Goal Pose
  -> /goal_pose
  -> planner_server/compute_path_to_pose
  -> planned nav_msgs/Path
  -> controller_server/follow_path
  -> MPPI
  -> /cmd_vel
```

현재 `mppi_controller.yaml`의 `global_costmap`은 `odom` 기준 rolling window와 footprint/inflation 설정만 포함한다. 따라서 planner 연결 구조와 Ackermann 최소 회전반경 반영은 동작하지만, 실제 장애물 회피까지 하려면 이후 `global_costmap`과 `local_costmap`에 지도 또는 센서 기반 obstacle layer를 추가해야 한다.

### 5.2 CSV 또는 사전 생성 Path 추종 모드

CSV 경로는 `x, y, yaw` 또는 header가 있는 `x,y,yaw` 형식으로 준비한다. `yaw`가 없으면 0으로 처리할 수 있지만, 곡선 경로에서는 yaw를 경로 접선 방향으로 채워두는 것이 좋다.

예시:

```csv
x,y,yaw
0.0,0.0,0.0
0.5,0.0,0.0
1.0,0.1,0.1
1.5,0.3,0.2
```

두 가지 입력 방식이 가능하다.

| 방식 | 입력 |
| :--- | :--- |
| CSV 파일 | `mppi_path_client.csv_file_path` 파라미터 |
| Path topic | `/mppi/csv_path` (`nav_msgs/Path`) |

`/mppi/csv_path`를 쓰면 외부 경로 생성 노드가 `nav_msgs/Path`를 발행하고, `mppi_path_client`가 이를 받아 바로 `FollowPath`로 보낸다. 초기 기준 frame은 `odom`이다.

---

## 6. 프레임 설계

### 6.1 1차 검증: `odom` 프레임 기반

초기 검증에서는 모든 것을 `odom` 기준으로 통일한다.

| 항목 | 설정 |
| :--- | :--- |
| 오도메트리 frame | `odom` |
| 차량 base frame | `base_link` |
| local costmap global frame | `odom` |
| FollowPath path frame | `odom` |
| TF | `odom -> base_link` |

장점:

- 전역 위치 보정 없이 바로 MPPI 제어 검증 가능
- `/odom/ekf_encoder_imu`가 제공하는 연속적인 상태만 사용하므로 제어 입력이 튀지 않음
- 경로도 `odom` 기준으로 만들면 TF 구조가 단순함

단점:

- 장시간 주행하면 누적 오차가 생김
- 절대 좌표 기반 지도/경로와 직접 연결하려면 별도 전역 보정이 필요함

### 6.2 확장 단계: `map -> odom -> base_link`

전역 경로 또는 지도 기반 주행이 필요해지면 `map -> odom` TF를 추가한다.

```text
map -> odom -> base_link
```

이때도 `/odom/ekf_encoder_imu`는 계속 로컬 필터로 유지한다. 전역 보정은 `map -> odom`에만 반영하고, `odom -> base_link`는 제어 안정성을 위해 연속적으로 유지한다.

---

## 7. 개발 패키지 구성

현재 워크스페이스 안에 `mppi_bringup_pkg` 패키지를 추가한다.

```text
src/
├── navigation2/                  # Nav2 Humble 소스
├── odom_pkg/                     # /odom/ekf_encoder_imu 제공
└── mppi_bringup_pkg/
    ├── package.xml
    ├── setup.py
    ├── config/
    │   └── mppi_controller.yaml
    ├── launch/
    │   └── mppi_controller.launch.py
    └── mppi_bringup_pkg/
        └── mppi_path_client.py
```

각 파일의 역할:

| 파일 | 역할 |
| :--- | :--- |
| `mppi_controller.yaml` | `planner_server`, `controller_server`, MPPI, costmap, path client 파라미터 |
| `mppi_controller.launch.py` | `planner_server`, `controller_server`, lifecycle manager, path client 실행 |
| `mppi_path_client.py` | `/goal_pose`를 planner action으로 보내고, planner/CSV path를 `FollowPath` action으로 전송 |

---
## 8. 실행 절차

### Step 1. 환경 source

빌드와 실행 전에는 `sv`, `sr` 명령어로 Python venv와 ROS 2 Humble 환경을 source한다.

```bash
sv
sr
```

Nav2를 소스 빌드한 뒤에는 워크스페이스 install도 source한다.

```bash
si
```

### Step 2. `/odom/ekf_encoder_imu` 실행

먼저 로컬 오도메트리 노드를 실행한다.

```bash
ros2 run odom_pkg ekf_encoder_imu_odometry
```

### Step 3. controller server 실행

`controller_server`는 lifecycle node이므로 lifecycle manager와 함께 실행해야 한다.

권장 launch 구성:

```text
mppi_controller.launch.py
  ├─ nav2_planner/planner_server
  ├─ nav2_controller/controller_server
  └─ nav2_lifecycle_manager/lifecycle_manager
```

lifecycle manager에는 최소 다음 설정을 넣는다.

```yaml
autostart: true
node_names: ["planner_server", "controller_server"]
```

RViz2 `2D Goal Pose`를 사용할 때는 기본값 그대로 실행한다. 이 경우 launch가 `planner_server`, `controller_server`, lifecycle manager와 함께 `mppi_path_client`도 실행한다.

```bash
ros2 launch mppi_bringup_pkg mppi_controller.launch.py
```

CSV 파일을 실행 직후 자동 전송할 때는 기본 launch의 `mppi_path_client`를 끄고, CSV용 `mppi_path_client`를 별도 터미널에서 하나만 실행한다.

```bash
ros2 launch mppi_bringup_pkg mppi_controller.launch.py start_path_client:=false
```

이렇게 분리하는 이유는 `mppi_path_client`가 `FollowPath` action goal을 보내는 주체이기 때문이다. 기본 launch의 path client와 CSV용 path client가 동시에 실행되면 `/goal_pose`, `/mppi/csv_path`, CSV 자동 전송 goal이 서로 다른 client에서 들어와 action 상태가 꼬일 수 있다.

### Step 4. 추종 경로 생성 또는 목표점 클릭

MPPI는 경로를 직접 만들지 않는다. 반드시 `nav_msgs/Path`를 만들어 `FollowPath` action으로 보내야 한다.

초기 검증용 경로는 `odom` 프레임 기준으로 만든다. 현재 개발 패키지 기준으로는 두 방법을 지원한다.

| 방법 | 사용법 |
| :--- | :--- |
| RViz2 목표 클릭 | RViz2 Fixed Frame을 `odom`으로 설정 후 `2D Goal Pose` 클릭 |
| 준비된 path 입력 | `/mppi/csv_path`에 `nav_msgs/Path` 발행 또는 `csv_file_path` 설정 |

```text
header.frame_id = "odom"
poses[0]        = 현재 차량 근처
poses[-1]       = 최종 goal
각 pose          = position + orientation 포함 권장
```

경로 품질 기준:

| 항목 | 권장 |
| :--- | :--- |
| waypoint 간격 | 0.1 m ~ 0.5 m |
| 곡선 구간 | 방향이 부드럽게 이어지도록 보간 |
| 첫 waypoint | 현재 차량 위치와 너무 멀지 않게 |
| frame | local costmap과 같은 `odom` |
| orientation | 가능하면 경로 접선 방향으로 채움 |

### Step 5. FollowPath action 전송

`mppi_path_client`가 다음 goal을 보낸다.

```python
from nav2_msgs.action import FollowPath

goal = FollowPath.Goal()
goal.path = path
goal.controller_id = "FollowPath"
goal.goal_checker_id = "goal_checker"
```

action server 이름:

```text
controller_server/follow_path
```

경로를 보내면 `controller_server`가 MPPI를 호출하고 `/cmd_vel`을 발행한다.


### Step 6. CSV 파일 자동 전송 예시

CSV 파일을 바로 읽어 MPPI에 보내려면 터미널을 두 개로 분리한다.

터미널 A에서 Nav2 planner/controller만 실행한다.

```bash
cd /home/hannibal/Mando2026_ws
sv
sr
si

ros2 launch mppi_bringup_pkg mppi_controller.launch.py start_path_client:=false
```

터미널 B에서 CSV 파일을 읽는 path client를 실행한다.

```bash
cd /home/hannibal/Mando2026_ws
sv
sr
si

ros2 run mppi_bringup_pkg mppi_path_client --ros-args \
  --params-file src/mppi_bringup_pkg/config/mppi_controller.yaml \
  -p csv_file_path:=/absolute/path/to/path.csv \
  -p auto_send_csv:=true
```

`auto_send_csv:=true`이면 path client가 시작되자마자 CSV 파일을 `nav_msgs/Path`로 변환하고 `FollowPath` action goal로 보낸다. 이때 CSV 좌표는 초기 검증 기준으로 `odom` 프레임 좌표여야 한다.

기본 launch를 이미 아래처럼 실행 중이라면 CSV용 path client를 추가로 실행하지 않는다.

```bash
ros2 launch mppi_bringup_pkg mppi_controller.launch.py
```

이 경우에는 이미 실행 중인 `mppi_path_client`가 있으므로, 준비된 `nav_msgs/Path`를 `/mppi/csv_path`로 발행하는 방식만 사용한다.

### Step 7. `/cmd_vel` 저수준 변환

Nav2 Humble의 controller server는 `/cmd_vel`로 `geometry_msgs/Twist`를 발행한다.

MPPI 출력:

```text
cmd_vel.linear.x   = 목표 전후방 속도
cmd_vel.angular.z  = 목표 yaw rate
```

차량형 플랫폼에서는 이를 실제 제어 입력으로 변환해야 한다.

기본 변환:

```text
조향각 delta = atan(wz * wheelbase / vx)
목표 속도 vx = 구동 제어기의 속도 목표
```

권장 구현:

| 입력 | 출력 |
| :--- | :--- |
| `/cmd_vel.linear.x` | 목표 속도 또는 구동 명령 |
| `/cmd_vel.angular.z` | 조향각 또는 조향 모터 목표 |
| `/odom/ekf_encoder_imu.twist.twist.linear.x` | 현재 속도 피드백 |

---
## 9. 튜닝 순서

처음부터 고속으로 실행하지 않는다. 아래 순서로 하나씩 올린다.

### 9.1 속도 제약

초기값:

```yaml
vx_max: 2.0
vx_min: -0.5
wz_max: 1.0
```

안정화 후:

```yaml
vx_max: 3.0
wz_max: vx_max / min_turning_r
```

`wz_max`는 최소 회전반경과 물리적으로 맞아야 한다.

```text
wz_max ≈ vx_max / min_turning_r
```

### 9.2 계산량

| 증상 | 조치 |
| :--- | :--- |
| control loop missed 경고 | `batch_size` 감소 또는 `visualize: false` |
| 궤적 품질 부족 | `batch_size` 증가 |
| 반응이 너무 짧음 | `time_steps` 증가 |
| 계산량 과다 | `time_steps` 감소 |

초기 권장:

```yaml
controller_frequency: 10.0
model_dt: 0.1
time_steps: 40
batch_size: 500
```

### 9.3 경로 추종 성향

| 증상 | 조정 |
| :--- | :--- |
| 경로에서 멀어짐 | `PathAlignCritic.cost_weight` 증가 |
| 목표로 잘 안 감 | `PathFollowCritic.cost_weight` 증가 |
| 커브 진입이 늦음 | `PathAngleCritic.cost_weight` 증가 |
| 조향이 떨림 | `wz_std` 감소, `temperature` 증가 |
| 너무 공격적임 | `temperature` 증가 |
| 너무 둔함 | `temperature` 감소 |

## 10. 핵심 결론

`/odom/ekf_encoder_imu`는 MPPI에 넣을 가장 중요한 로컬 상태 입력이다. 이 토픽은 `controller_server.odom_topic`으로 연결하여 현재 속도를 제공하고, 동시에 `odom -> base_link` TF를 통해 현재 pose 계산의 기반이 된다.

초기 성공 전략은 단순하다.

```text
1. /odom/ekf_encoder_imu + odom->base_link 안정화
2. local_costmap.global_frame = odom
3. RViz2 /goal_pose는 planner_server/compute_path_to_pose로 path 생성
4. /mppi/csv_path는 준비된 nav_msgs/Path를 그대로 FollowPath로 전송
5. FollowPath path.header.frame_id = odom
6. controller_server.odom_topic = /odom/ekf_encoder_imu
7. planner와 MPPI motion_model = Ackermann 계열 설정
8. /cmd_vel을 실제 차량 제어 명령으로 변환
```

이 흐름으로 먼저 `odom` 프레임 내 MPPI 제어를 성공시킨 뒤, 필요하면 `map -> odom` 전역 보정과 map frame 경로 추종으로 확장한다.
