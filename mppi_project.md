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

- `/odom/ekf_encoder_imu/path`는 지나온 궤적을 기록한 시각화용 path이다. MPPI가 따라가야 할 계획 경로로 쓰면 안 된다.
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

## 8. Nav2 planner/controller 설정 초안

파일 예시:

```text
src/mppi_bringup_pkg/config/mppi_controller.yaml
```

초기 검증용 최소 설정:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false

    controller_frequency: 10.0
    odom_topic: /odom/ekf_encoder_imu
    odom_duration: 0.3

    transform_tolerance: 0.3
    costmap_update_timeout: 0.30
    failure_tolerance: 1.5

    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001

    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
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
      batch_size: 500
      iteration_count: 1

      open_loop: false
      reset_period: 1.0
      retry_attempt_limit: 1

      vx_std: 0.3
      wz_std: 0.3

      vx_max: 2.0
      vx_min: -0.5
      wz_max: 1.0

      temperature: 0.3
      gamma: 0.015

      visualize: true
      regenerate_noises: true

      motion_model: "Ackermann"
      ackermann:
        plugin: "mppi::AckermannMotionModel"
        min_turning_r: 1.65

      TrajectoryValidator:
        plugin: "mppi::DefaultOptimalTrajectoryValidator"
        collision_lookahead_time: 2.0
        consider_footprint: true

      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3

      critics:
        [
          "ConstraintCritic",
          "CostCritic",
          "GoalCritic",
          "GoalAngleCritic",
          "PathAlignCritic",
          "PathFollowCritic",
          "PathAngleCritic",
          "PreferForwardCritic",
        ]

      ConstraintCritic:
        enabled: true
        cost_weight: 4.0
        cost_power: 1

      GoalCritic:
        enabled: true
        cost_weight: 5.0
        cost_power: 1
        threshold_to_consider: 2.0

      GoalAngleCritic:
        enabled: true
        cost_weight: 3.0
        cost_power: 1
        threshold_to_consider: 1.0
        symmetric_yaw_tolerance: false

      PathAlignCritic:
        enabled: true
        cost_weight: 14.0
        cost_power: 1
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false

      PathFollowCritic:
        enabled: true
        cost_weight: 5.0
        cost_power: 1
        offset_from_furthest: 5

      PathAngleCritic:
        enabled: true
        cost_weight: 2.0
        cost_power: 1
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.2
        forward_preference: true

      PreferForwardCritic:
        enabled: true
        cost_weight: 5.0
        cost_power: 1
        threshold_to_consider: 0.5

      CostCritic:
        enabled: true
        cost_weight: 3.81
        cost_power: 1
        consider_footprint: true
        collision_cost: 1000000.0
        critical_cost: 300.0
        near_goal_distance: 1.0

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false

      update_frequency: 10.0
      publish_frequency: 10.0

      global_frame: odom
      robot_base_frame: base_link
      transform_tolerance: 0.3

      rolling_window: true
      width: 20
      height: 20
      resolution: 0.1

      footprint: "[[1.15, 0.5], [1.15, -0.5], [-0.35, -0.4], [-0.35, 0.4]]"

      plugins: ["inflation_layer"]

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.8
        cost_scaling_factor: 3.0

      always_send_full_costmap: true
```

### 8.1 `planner_server` 파라미터 설명

`planner_server`는 RViz2의 `/goal_pose` 같은 목표 pose를 실제 MPPI가 추종할 수 있는 `nav_msgs/Path`로 바꾸는 전역 경로 생성 서버다. 현재 구조에서는 `mppi_path_client`가 `/goal_pose`를 받으면 `planner_server/compute_path_to_pose` action을 호출하고, planner가 만든 path를 다시 `controller_server/follow_path`로 전달한다.

현재 설정의 핵심은 `SmacPlannerHybrid`다. 이 planner는 Hybrid-A* 계열이라 단순 2D 직선 최단거리만 보는 것이 아니라, 차량형 플랫폼의 회전반경과 heading을 포함한 경로를 만들 수 있다.

| 파라미터 | 현재 값 | 의미 |
| :--- | :--- | :--- |
| `use_sim_time` | `false` | `/clock` 기반 시간 사용 여부. 실차/로컬 실행에서는 보통 `false` |
| `expected_planner_frequency` | `1.0` | planner가 기대하는 계획 생성 빈도. 너무 높게 잡으면 큰 costmap이나 Hybrid-A*에서 부담이 커질 수 있음 |
| `planner_plugins` | `["GridBased"]` | planner plugin ID 목록. action 요청의 `planner_id`와 연결됨 |
| `GridBased.plugin` | `nav2_smac_planner/SmacPlannerHybrid` | 실제 경로 생성 알고리즘. Ackermann 차량형 경로에 적합 |
| `tolerance` | `0.5` | 정확한 goal pose에 도달하는 path를 못 찾을 때 허용하는 목표 근처 오차 |
| `downsample_costmap` | `false` | planning costmap 해상도를 낮춰 속도를 높일지 여부 |
| `downsampling_factor` | `1` | downsample을 켰을 때 해상도를 몇 배 키울지 결정 |
| `allow_unknown` | `true` | unknown 영역 통과 허용 여부. 현재는 map 기반 unknown 처리가 약하므로 초기 검증에서는 `true` |
| `max_iterations` | `1000000` | 탐색 최대 반복 횟수. 목표가 불가능할 때 무한 탐색을 막는 안전장치 |
| `max_on_approach_iterations` | `1000` | goal tolerance 안에 들어온 뒤 최종 접근을 더 시도하는 반복 횟수 |
| `max_planning_time` | `5.0` | planning, smoothing까지 포함한 최대 계획 시간 |
| `motion_model_for_search` | `DUBIN` | 전진 주행 중심의 Dubins motion model. 후진까지 적극 허용하려면 `REEDS_SHEPP` 계열을 검토 |
| `angle_quantization_bins` | `72` | heading 각도를 몇 개 bin으로 나눠 탐색할지 결정. 클수록 방향 표현은 세밀하지만 계산량 증가 |
| `analytic_expansion_ratio` | `3.5` | goal 근처에서 곡선 shortcut을 시도하는 빈도/비율 |
| `analytic_expansion_max_length` | `15.0` | analytic expansion으로 허용할 최대 연결 길이. 최소 회전반경보다 충분히 크게 둬야 함 |
| `minimum_turning_radius` | `1.65` | planner가 가정하는 최소 회전반경. MPPI의 `ackermann.min_turning_r`와 맞추는 핵심 값 |
| `reverse_penalty` | `2.1` | 후진 motion 비용. `DUBIN`에서는 영향이 제한적이고, `REEDS_SHEPP`에서 중요 |
| `change_penalty` | `0.0` | 전진/후진 방향 전환 비용 |
| `non_straight_penalty` | `1.2` | 곡선 motion에 추가 비용을 줘 불필요한 굽은 경로를 줄임 |
| `cost_penalty` | `2.0` | costmap 비용이 높은 영역을 피하려는 정도 |
| `retrospective_penalty` | `0.025` | 탐색 효율을 위해 늦은 maneuver를 약간 선호하는 비용 |
| `lookup_table_size` | `20.0` | Dubins/Reeds-Shepp 거리 계산 lookup table 크기 |
| `cache_obstacle_heuristic` | `false` | 같은 목표로 반복 replanning할 때 obstacle heuristic 캐시 사용 여부 |
| `smooth_path` | `true` | Hybrid-A* 결과 path를 smoother로 후처리할지 여부 |
| `smoother.*` | `max_iterations`, `w_smooth`, `w_data` 등 | path smoothing 강도와 반복 조건 |

중요한 연결 관계:

```text
mppi_path_client.planner_id = GridBased
planner_server.planner_plugins = ["GridBased"]
GridBased.plugin = nav2_smac_planner/SmacPlannerHybrid
```

따라서 `mppi_path_client`가 `planner_id: GridBased`로 요청하면, `planner_server`는 `SmacPlannerHybrid`를 사용해 path를 생성한다.

### 8.2 `controller_server` 파라미터 설명

`controller_server`는 planner가 만든 `nav_msgs/Path`를 받아 실제 제어 명령인 `/cmd_vel`을 생성하는 lifecycle node다. MPPI는 별도 실행 파일이 아니라 `controller_server` 안에 `FollowPath` controller plugin으로 로드된다.

상위 제어 흐름:

```text
planned nav_msgs/Path
  -> controller_server/follow_path
  -> FollowPath plugin = nav2_mppi_controller::MPPIController
  -> /odom/ekf_encoder_imu + local_costmap + critics
  -> /cmd_vel
```

#### controller server 기본 파라미터

| 파라미터 | 현재 값 | 의미 |
| :--- | :--- | :--- |
| `use_sim_time` | `false` | `/clock` 사용 여부 |
| `controller_frequency` | `10.0` | MPPI 제어 루프 주기. 10Hz면 0.1초마다 새 `/cmd_vel` 계산 |
| `odom_topic` | `/odom/ekf_encoder_imu` | MPPI가 현재 속도를 읽는 odometry 입력. 여기서는 local filter 역할 |
| `odom_duration` | `0.3` | odom 데이터 유효 시간. 이 시간보다 오래된 odom은 신뢰하지 않음 |
| `transform_tolerance` | `0.3` | TF 시간 오차 허용치 |
| `costmap_update_timeout` | `0.30` | local costmap 업데이트를 기다리는 최대 시간 |
| `failure_tolerance` | `1.5` | controller 실패를 어느 정도 시간까지 허용할지 결정 |
| `min_x_velocity_threshold` | `0.001` | 이보다 작은 x 속도는 0에 가깝다고 취급 |
| `min_theta_velocity_threshold` | `0.001` | 이보다 작은 yaw rate는 0에 가깝다고 취급 |
| `progress_checker_plugins` | `["progress_checker"]` | 주행 중 실제로 진행하고 있는지 판단하는 plugin 목록 |
| `goal_checker_plugins` | `["goal_checker"]` | 목표 도달 여부를 판단하는 plugin 목록 |
| `controller_plugins` | `["FollowPath"]` | controller plugin ID 목록. MPPI는 `FollowPath` ID로 로드됨 |

#### progress checker / goal checker

| 파라미터 | 현재 값 | 의미 |
| :--- | :--- | :--- |
| `progress_checker.plugin` | `nav2_controller::SimpleProgressChecker` | 일정 시간 안에 충분히 움직였는지 검사 |
| `required_movement_radius` | `0.5` | 진행했다고 인정할 최소 이동 거리 |
| `movement_time_allowance` | `10.0` | 위 이동 거리를 만족해야 하는 제한 시간 |
| `goal_checker.plugin` | `nav2_controller::SimpleGoalChecker` | 목표 도달 판정 plugin |
| `stateful` | `true` | goal checker가 내부 상태를 유지하며 도달 판정 |
| `xy_goal_tolerance` | `0.5` | goal 위치 허용 오차 |
| `yaw_goal_tolerance` | `0.3` | goal heading 허용 오차 |

#### MPPI 기본 샘플링 파라미터

| 파라미터 | 현재 값 | 의미 |
| :--- | :--- | :--- |
| `FollowPath.plugin` | `nav2_mppi_controller::MPPIController` | `FollowPath` controller ID가 실제로 로드할 MPPI plugin |
| `time_steps` | `40` | 하나의 후보 궤적이 미래 몇 step까지 예측되는지 |
| `model_dt` | `0.1` | 예측 step 간 시간 간격. 현재 예측 horizon은 `40 * 0.1 = 4.0초` |
| `batch_size` | `500` | 한 control cycle에서 샘플링할 후보 궤적 수. 클수록 탐색은 풍부하지만 CPU 부하 증가 |
| `iteration_count` | `1` | 한 제어 주기 안에서 최적화 반복 횟수 |
| `open_loop` | `false` | 이전 control sequence를 open-loop로 계속 쓰지 않고 odom feedback을 반영 |
| `reset_period` | `1.0` | noise/control sequence를 주기적으로 reset하는 시간 |
| `retry_attempt_limit` | `1` | 실패 시 재시도 제한 |
| `vx_std` | `0.3` | 선속도 샘플링 noise 표준편차 |
| `wz_std` | `0.3` | 각속도 샘플링 noise 표준편차 |
| `vx_max` | `2.0` | 최대 전진 속도 |
| `vx_min` | `-0.5` | 최소 x 속도. 음수면 제한적으로 후진 후보도 가능 |
| `wz_max` | `1.0` | 최대 yaw rate |
| `temperature` | `0.3` | 비용이 낮은 후보를 얼마나 강하게 선택할지 결정 |
| `gamma` | `0.015` | control effort와 smoothing에 영향을 주는 MPPI 비용 계수 |
| `visualize` | `true` | `/trajectories`, `/transformed_global_plan` 시각화 발행 |
| `regenerate_noises` | `true` | 매 iteration마다 noise를 새로 생성 |

#### Ackermann motion model

| 파라미터 | 현재 값 | 의미 |
| :--- | :--- | :--- |
| `motion_model` | `Ackermann` | 차량형 운동 모델 사용 |
| `ackermann.plugin` | `mppi::AckermannMotionModel` | MPPI 내부 Ackermann motion model plugin |
| `ackermann.min_turning_r` | `1.65` | MPPI rollout이 따르는 최소 회전반경. planner의 `minimum_turning_radius`와 맞춰야 함 |

planner와 controller의 회전반경이 다르면 planner는 돌 수 있다고 판단한 경로를 MPPI가 추종하지 못하거나, 반대로 MPPI는 가능한데 planner가 지나치게 넓게 도는 path를 만들 수 있다. 실제 차량의 최소 회전반경이 `1.65m`이므로 현재는 둘 다 `1.65m`로 맞춘다.

#### trajectory validator / visualizer

| 파라미터 | 현재 값 | 의미 |
| :--- | :--- | :--- |
| `TrajectoryValidator.plugin` | `mppi::DefaultOptimalTrajectoryValidator` | 최적 후보 궤적이 충돌/제약 조건을 만족하는지 최종 검사 |
| `collision_lookahead_time` | `2.0` | 미래 몇 초까지 충돌 가능성을 검사할지 |
| `consider_footprint` | `true` | 점 로봇이 아니라 footprint polygon 기준으로 충돌 검사 |
| `TrajectoryVisualizer.trajectory_step` | `5` | 후보 궤적 시각화에서 몇 번째 trajectory마다 표시할지 |
| `TrajectoryVisualizer.time_step` | `3` | 궤적의 time step을 얼마나 건너뛰어 표시할지 |

#### critics

MPPI critic은 후보 궤적마다 비용을 매기고, 비용이 낮은 후보가 최종 `/cmd_vel`에 더 크게 반영되도록 한다.

| critic | 역할 |
| :--- | :--- |
| `ConstraintCritic` | 속도, 회전, 운동학적 제약을 벗어나는 후보를 억제 |
| `CostCritic` | local costmap 비용과 footprint 충돌 위험을 반영 |
| `GoalCritic` | goal 위치에 가까운 후보를 선호 |
| `GoalAngleCritic` | goal heading에 가까운 후보를 선호 |
| `PathAlignCritic` | 후보 궤적이 global path 방향과 잘 정렬되도록 유도 |
| `PathFollowCritic` | 후보가 path를 따라 전진하도록 유도 |
| `PathAngleCritic` | path heading과 차량 heading의 차이를 줄임 |
| `PreferForwardCritic` | 후진보다 전진 후보를 선호 |

주요 critic 파라미터 해석:

| 파라미터 패턴 | 의미 |
| :--- | :--- |
| `enabled` | 해당 critic 사용 여부 |
| `cost_weight` | 이 critic 비용의 영향력. 클수록 해당 기준을 강하게 따름 |
| `cost_power` | 비용을 선형/비선형으로 키우는 정도 |
| `threshold_to_consider` | goal 근처 등 특정 거리 조건 안팎에서 critic을 적용할지 결정 |
| `offset_from_furthest` | path 위의 어느 지점까지 앞을 보고 평가할지 결정 |
| `consider_footprint` | cost 평가 때 차량 footprint 전체를 고려할지 여부 |

튜닝 방향은 단순하다. path는 잘 따라가지만 장애물이나 가장자리에 너무 붙으면 `CostCritic`과 inflation 설정을 키우고, path를 자주 벗어나면 `PathAlignCritic`, `PathFollowCritic` 영향력을 조정한다. 반대로 진동이 크면 critic weight, `vx_std`, `wz_std`, `temperature`를 함께 낮춰 후보 분포를 안정화한다.

주의:

- `use_sim_time`은 실제 `/clock`을 쓰는 환경이면 `true`, 일반 실차/로컬 실행이면 `false`로 둔다.
- `model_dt`는 `1 / controller_frequency`와 맞춘다. 예: `controller_frequency: 10.0`이면 `model_dt: 0.1`.
- `motion_model` 값은 `"Ackermann"`처럼 첫 글자를 대문자로 쓴다.
- `min_turning_r`, `footprint`, `vx_max`, `wz_max`는 실제 차량 제원에 맞춰 반드시 수정한다.

---

## 9. 실행 절차

### Step 1. 환경 source

빌드와 실행 전에는 `sv`, `sr` 명령어로 Python venv와 ROS 2 Humble 환경을 source한다.

```bash
sv
sr
```

Nav2를 소스 빌드한 뒤에는 워크스페이스 install도 source한다.

```bash
source /home/hannibal/Mando2026_ws/install/setup.bash
```

### Step 2. `/odom/ekf_encoder_imu` 실행

먼저 로컬 오도메트리 노드를 실행한다.

```bash
ros2 run odom_pkg ekf_encoder_imu_odometry
```

확인:

```bash
ros2 topic echo /odom/ekf_encoder_imu --once
ros2 topic hz /odom/ekf_encoder_imu
ros2 run tf2_ros tf2_echo odom base_link
```

정상 조건:

| 항목 | 기대값 |
| :--- | :--- |
| topic type | `nav_msgs/msg/Odometry` |
| `header.frame_id` | `odom` |
| `child_frame_id` | `base_link` |
| TF | `odom -> base_link` 존재 |
| rate | MPPI 제어 주기보다 높음. 권장 30 Hz 이상 |

### Step 3. Nav2 소스 빌드

`src/navigation2`의 Humble 소스를 사용한다.

```bash
cd /home/hannibal/Mando2026_ws
sv
sr

rosdep install --from-paths src/navigation2 src/mppi_bringup_pkg --ignore-src -r -y

colcon build --symlink-install \
  --base-paths src/navigation2 src/mppi_bringup_pkg \
  --packages-up-to \
    nav2_controller \
    nav2_mppi_controller \
    nav2_planner \
    nav2_smac_planner \
    nav2_lifecycle_manager \
    mppi_bringup_pkg \
  --allow-overriding \
    nav_2d_msgs \
    nav_2d_utils \
    nav2_common \
    nav2_msgs \
    nav2_util \
    nav2_core \
    nav2_costmap_2d \
    nav2_map_server \
    nav2_voxel_grid \
    nav2_controller \
    nav2_mppi_controller \
    nav2_planner \
    nav2_smac_planner \
    nav2_lifecycle_manager \
  --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash
```

설치 확인:

```bash
ros2 pkg prefix nav2_mppi_controller
ros2 pkg executables nav2_controller
```

주의:

- 워크스페이스 안에 다른 Nav2 복사본이 있으면 `colcon`이 같은 패키지를 두 번 발견할 수 있다. 따라서 위처럼 `--base-paths src/navigation2 src/mppi_bringup_pkg`을 반드시 붙여 현재 프로젝트의 Nav2만 discovery 하도록 제한한다.
- `nav2_util` 테스트는 `test_msgs` 같은 테스트 전용 의존성을 요구한다. MPPI 실행에는 테스트 타깃이 필요 없으므로 `--cmake-args -DBUILD_TESTING=OFF`로 끈다.
- `--packages-up-to`가 끌고 오는 `nav_2d_msgs`, `nav_2d_utils`, `nav2_map_server`, `nav2_voxel_grid`도 `/opt/ros/humble`에 이미 있으므로 override 경고를 없애려면 `--allow-overriding` 목록에 함께 넣는다.

### Step 4. controller server 실행

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

실행:

```bash
ros2 launch mppi_bringup_pkg mppi_controller.launch.py
```

확인:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 action info /follow_path
ros2 action info /compute_path_to_pose
ros2 topic list | grep cmd_vel
```

정상 조건:

| 확인 항목 | 기대값 |
| :--- | :--- |
| lifecycle | `/planner_server`, `/controller_server` 모두 `active [3]` |
| action server | `/compute_path_to_pose`, `/follow_path` server 각각 1개 |
| output topic | `/cmd_vel` 존재 |
| odom subscription | `/odom/ekf_encoder_imu` 구독 중 |

### Step 5. 추종 경로 생성 또는 목표점 클릭

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

### Step 6. FollowPath action 전송

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

확인:

```bash
ros2 action info /follow_path
ros2 topic echo /cmd_vel
ros2 topic echo /trajectories
ros2 topic echo /mppi/status
ros2 topic echo /mppi/active_path
```

### Step 7. CSV 파일 자동 전송 예시

CSV 파일을 바로 읽어 MPPI에 보내려면 launch 후 별도 실행으로 다음처럼 사용할 수 있다.

```bash
ros2 run mppi_bringup_pkg mppi_path_client --ros-args \
  --params-file src/mppi_bringup_pkg/config/mppi_controller.yaml \
  -p csv_file_path:=/absolute/path/to/path.csv \
  -p auto_send_csv:=true
```

이때 CSV 좌표는 초기 검증 기준으로 `odom` 프레임 좌표여야 한다.

### Step 8. `/cmd_vel` 저수준 변환

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

## 10. 검증 체크리스트

### 10.1 오도메트리 검증

```bash
ros2 topic echo /odom/ekf_encoder_imu --once
ros2 topic hz /odom/ekf_encoder_imu
ros2 run tf2_ros tf2_echo odom base_link
```

확인:

- `linear.x`가 실제 전후진 방향과 일치하는가
- 좌회전/우회전 시 `angular.z` 부호가 ROS 표준과 일치하는가
- 정지 시 twist가 0 근처로 안정되는가
- TF가 끊기지 않는가

### 10.2 Nav2 연결 검증

```bash
ros2 lifecycle get /controller_server
ros2 action info /follow_path
ros2 node info /controller_server
```

확인:

- `/controller_server`가 active인가
- `/follow_path` action server가 살아 있는가
- `/controller_server`가 `/odom/ekf_encoder_imu`를 구독하는가
- `/local_costmap/costmap`이 발행되는가

### 10.3 MPPI 출력 검증

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /trajectories
```

확인:

- 경로를 보내기 전에는 `/cmd_vel`이 0이거나 발행되지 않는가
- 경로를 보낸 뒤 `linear.x`, `angular.z`가 생성되는가
- `/trajectories`가 경로 주변에 분포하는가
- 제어 주기 missed warning이 반복되지 않는가

### 10.4 RViz 확인 항목

RViz에서 다음을 표시한다.

| Display | Topic / Frame |
| :--- | :--- |
| TF | `odom`, `base_link` |
| Odometry | `/odom/ekf_encoder_imu` |
| Path | MPPI에 보낸 계획 경로 |
| Local costmap | `/local_costmap/costmap` |
| MPPI trajectories | `/trajectories` |

### 10.5 `rviz/mppi.rviz` 시각화 토픽 상세

`rviz/mppi.rviz`는 MPPI 제어 흐름을 한 화면에서 확인하기 위한 전용 RViz 설정 파일이다. 실행은 다음처럼 한다.

```bash
rviz2 -d /home/hannibal/Mando2026_ws/rviz/mppi.rviz
```

이 설정의 `Fixed Frame`은 `odom`이다. 현재 1차 검증 구조가 `odom -> base_link`만으로 MPPI를 돌리는 방식이므로, RViz의 모든 path, costmap, footprint, trajectory를 `odom` 기준에서 보는 것이 가장 단순하고 안전하다.

```text
Fixed Frame = odom

odom
  └── base_link
```

#### 전체 표시 흐름

RViz에 표시되는 토픽은 다음 세 종류로 나누어 해석한다.

```text
1. 실제 차량 상태
   /odom/ekf_encoder_imu
   /odom/ekf_encoder_imu/path
   TF: odom, base_link

2. MPPI 입력/내부 상태
   /goal_pose
   /mppi/csv_path
   /mppi/active_path
   /transformed_global_plan

3. MPPI 계산 결과와 환경 비용
   /trajectories
   /local_costmap/costmap
   /local_costmap/costmap_updates
   /local_costmap/published_footprint
```

가장 중요한 구분은 아래 세 path의 의미가 서로 다르다는 점이다.

| 토픽 | 의미 | MPPI 입력 여부 |
| :--- | :--- | :---: |
| `/odom/ekf_encoder_imu/path` | 차량이 이미 지나온 실제 추정 궤적 | 아니오 |
| `/mppi/active_path` | `mppi_path_client`가 MPPI에 보낸 전체 목표 경로 | 예 |
| `/transformed_global_plan` | MPPI 내부에서 현재 제어에 쓰는 잘린 경로 조각 | 내부 결과 |

#### `Grid`

| 항목 | 값 |
| :--- | :--- |
| Display type | `rviz_default_plugins/Grid` |
| 기준 frame | `<Fixed Frame>` = `odom` |
| 목적 | `odom` 평면의 거리 감각 확인 |

`Grid`는 ROS 토픽을 구독하지 않는 RViz 자체 표시 요소다. 차량, path, costmap이 `odom` 평면 위에서 어디에 놓이는지 보기 위한 배경 격자다.

#### `Axes: base_link`

| 항목 | 값 |
| :--- | :--- |
| Display type | `rviz_default_plugins/Axes` |
| Reference Frame | `base_link` |
| 출처 | TF `odom -> base_link` |
| 목적 | 차량 현재 위치와 heading 확인 |

`base_link` 축은 차량의 물리 기준 프레임이다. X축은 차량 전방, Y축은 좌측, Z축은 위쪽이다. MPPI가 보는 현재 차량 pose가 정상인지 가장 직관적으로 확인할 수 있다.

정상 상태:

- 차량을 앞으로 움직이면 `base_link` X축 방향으로 궤적이 늘어난다.
- 좌회전하면 `base_link` yaw가 ROS 표준 방향으로 회전한다.
- `/odom/ekf_encoder_imu/path` 끝점과 `base_link` 위치가 거의 일치한다.

문제 징후:

- `base_link`가 보이지 않으면 `odom -> base_link` TF가 없거나 RViz fixed frame이 맞지 않는 것이다.
- 차량이 실제 회전 방향과 반대로 돈다면 IMU yaw 또는 yaw-rate 부호를 확인해야 한다.

#### `Axes: odom`

| 항목 | 값 |
| :--- | :--- |
| Display type | `rviz_default_plugins/Axes` |
| Reference Frame | `odom` |
| 출처 | RViz fixed frame |
| 목적 | 로컬 좌표계 원점 확인 |

`odom`은 로컬 주행 기준점이다. `/odom/ekf_encoder_imu`는 이 프레임 안에서 `base_link`가 어떻게 움직였는지를 표현한다. 초기 MPPI 검증에서는 모든 경로와 costmap도 `odom` 기준으로 맞춘다.

#### `Encoder IMU Path`

| 항목 | 값 |
| :--- | :--- |
| Display type | `rviz_default_plugins/Path` |
| Topic | `/odom/encoder_imu/path` |
| Message type | `nav_msgs/Path` |
| 출처 | `encoder_imu_odometry` |
| 의미 | raw encoder+IMU dead-reckoning 궤적 |

이 path는 EKF를 거치지 않은 raw dead-reckoning 결과다. MPPI 제어 입력으로 쓰는 주 경로는 아니고, EKF 결과와 비교하기 위한 참고 궤적이다.

활용:

- `/odom/encoder_imu/path`와 `/odom/ekf_encoder_imu/path`가 크게 벌어지는지 비교한다.
- raw 적분이 노이즈나 yaw 드리프트에 얼마나 민감한지 확인한다.

주의:

- 이 토픽은 “따라갈 목표 경로”가 아니다.
- MPPI에 넣을 계획 경로로 사용하지 않는다.

#### `EKF Encoder IMU Path`

| 항목 | 값 |
| :--- | :--- |
| Display type | `rviz_default_plugins/Path` |
| Topic | `/odom/ekf_encoder_imu/path` |
| Message type | `nav_msgs/Path` |
| 출처 | `ekf_encoder_imu_odometry` |
| 의미 | EKF local filter가 추정한 실제 주행 궤적 |

이 토픽은 `/odom/ekf_encoder_imu` 오도메트리 pose를 누적한 시각화용 path다. 현재 차량이 실제로 어떻게 이동했다고 추정되는지 보여준다.

정상 상태:

- `base_link`가 이 path의 끝점 근처에 있다.
- 정지 중에는 path가 새로 길게 늘어나지 않는다.
- 직진 시 path가 큰 좌우 진동 없이 뻗는다.

중요:

```text
/odom/ekf_encoder_imu/path = 과거 주행 기록
/mppi/active_path          = 앞으로 따라갈 목표 경로
```

이 둘을 헷갈리면 안 된다. MPPI가 따라가야 할 것은 `/mppi/active_path` 또는 `FollowPath`로 보낸 계획 경로다.

#### `Pose: /goal_pose`

| 항목 | 값 |
| :--- | :--- |
| Display type | `rviz_default_plugins/Pose` |
| Topic | `/goal_pose` |
| Message type | `geometry_msgs/PoseStamped` |
| 출처 | RViz2 `2D Goal Pose` tool |
| 소비자 | `mppi_path_client` |

RViz에서 `2D Goal Pose`를 클릭하면 `/goal_pose`가 발행된다. `mppi_path_client`는 이 목표 pose를 `planner_server`의 `ComputePathToPose` action으로 보내고, planner가 계산한 `nav_msgs/Path`를 MPPI에 전달한다.

흐름:

```text
RViz2 2D Goal Pose 클릭
  -> /goal_pose
  -> mppi_path_client
  -> planner_server/compute_path_to_pose
  -> planned nav_msgs/Path
  -> /mppi/active_path
  -> FollowPath action
  -> controller_server
  -> MPPI
```

주의:

- `/goal_pose` 자체는 MPPI가 직접 따라가는 입력이 아니다.
- MPPI는 path follower이므로 `/goal_pose`는 반드시 path로 변환되어야 한다.
- 현재 기본 설정에서는 이 변환을 `SmacPlannerHybrid`가 수행한다.
- 현재 설정에서는 `/goal_pose.header.frame_id`가 `odom`이어야 한다. RViz Fixed Frame을 `odom`으로 둔다.

#### `Path: /mppi/active_path`

| 항목 | 값 |
| :--- | :--- |
| Display type | `rviz_default_plugins/Path` |
| Topic | `/mppi/active_path` |
| Message type | `nav_msgs/Path` |
| 출처 | `mppi_path_client` |
| 의미 | MPPI에 전송한 활성 목표 경로 |

`/mppi/active_path`는 현재 MPPI에게 “이 경로를 따라가라”고 보낸 path를 확인하기 위한 토픽이다.

생성 경로:

- RViz `/goal_pose` 입력을 planner가 계산한 path로 변환
- `/mppi/csv_path`로 들어온 path를 그대로 활성 path로 채택
- `csv_file_path` 파라미터로 읽은 CSV path를 활성 path로 채택

정상 상태:

- RViz 목표 클릭 직후 planner가 계산한 녹색 path가 현재 차량 위치에서 목표점까지 생성된다.
- path의 시작점이 `base_link` 근처에 있다.
- path frame은 `odom`이다.

문제 징후:

- `/goal_pose`는 발행되는데 `/mppi/active_path`가 안 생기면 `mppi_path_client`가 goal을 거부했을 가능성이 있다.
- path가 엉뚱한 위치에 있으면 frame이 `odom`이 아니거나 좌표계가 잘못된 것이다.

#### `Path: /mppi/csv_path`

| 항목 | 값 |
| :--- | :--- |
| Display type | `rviz_default_plugins/Path` |
| Topic | `/mppi/csv_path` |
| Message type | `nav_msgs/Path` |
| 출처 | 외부 path publisher |
| 소비자 | `mppi_path_client` |

`/mppi/csv_path`는 준비된 경로를 MPPI에 넣기 위한 입력 토픽이다. 이름에 `csv`가 들어가지만 메시지는 CSV 파일이 아니라 이미 변환된 `nav_msgs/Path`다.

흐름:

```text
CSV 변환 노드 또는 path 생성 노드
  -> /mppi/csv_path
  -> mppi_path_client
  -> /mppi/active_path
  -> FollowPath action
```

정상 조건:

- `path.header.frame_id = "odom"`
- `poses`가 최소 2개 이상
- waypoint 간격이 너무 듬성듬성하지 않음
- 각 pose의 orientation이 가능하면 경로 접선 방향과 일치

주의:

- `/odom/ekf_encoder_imu/path`처럼 지나온 경로를 넣는 용도가 아니다.
- 앞으로 따라갈 계획 경로만 넣는다.

#### `Path: /transformed_global_plan`

| 항목 | 값 |
| :--- | :--- |
| Display type | `rviz_default_plugins/Path` |
| Topic | `/transformed_global_plan` |
| Message type | `nav_msgs/Path` |
| 출처 | `nav2_mppi_controller` |
| 의미 | MPPI 내부에서 현재 제어에 사용하는 local plan 조각 |

`/transformed_global_plan`은 MPPI가 받은 전체 path를 현재 로봇 pose 근처 기준으로 잘라내고 변환한 결과다. 이름에는 `global_plan`이 들어가지만, 현재 1차 구조에서는 frame을 `odom`으로 통일해서 쓰고 있다.

비교:

```text
/mppi/active_path
  전체 목표 경로

/transformed_global_plan
  MPPI 내부 path handler가 현재 순간 실제로 보고 있는 경로 구간
```

정상 상태:

- `/mppi/active_path` 일부가 차량 주변에 잘려서 보인다.
- 차량이 움직이면 이미 지나간 waypoint가 pruning되며 앞쪽 경로만 남는다.

문제 징후:

- `/mppi/active_path`는 있는데 `/transformed_global_plan`이 비어 있으면 path frame, TF, path 시작점 거리 문제를 의심한다.
- 경로가 갑자기 사라지면 `max_robot_pose_search_dist`, `prune_distance`, TF 상태를 확인한다.

#### `MarkerArray: /trajectories`

| 항목 | 값 |
| :--- | :--- |
| Display type | `rviz_default_plugins/MarkerArray` |
| Topic | `/trajectories` |
| Message type | `visualization_msgs/MarkerArray` |
| 출처 | `nav2_mppi_controller` |
| 의미 | MPPI가 샘플링한 후보 rollout 궤적 |

MPPI는 하나의 제어 입력만 계산하지 않는다. 여러 개의 후보 속도/회전 명령 시퀀스를 샘플링하고, 각 후보를 미래로 전개한 뒤 cost가 가장 낮은 제어를 선택한다. 이 후보 궤적들이 `/trajectories`로 표시된다.

관련 파라미터:

```yaml
FollowPath:
  visualize: true
  batch_size: 500
  time_steps: 40
  model_dt: 0.1

  TrajectoryVisualizer:
    trajectory_step: 5
    time_step: 3
```

해석:

```text
예측 시간 = time_steps * model_dt = 40 * 0.1 = 4.0초
```

정상 상태:

- 목표 path 방향으로 후보 궤적들이 부채꼴처럼 펼쳐진다.
- costmap 장애물이나 경로 방향에 따라 후보 분포가 달라진다.
- `/cmd_vel`이 나오는 동안 주기적으로 갱신된다.

문제 징후:

- `/trajectories`가 전혀 안 나오면 `FollowPath` goal이 아직 들어가지 않았거나 `visualize: false`일 수 있다.
- 후보 궤적이 너무 넓게 흔들리면 `vx_std`, `wz_std`가 크거나 `temperature` 튜닝이 필요할 수 있다.
- 후보가 물리적으로 불가능한 급회전을 많이 만들면 `min_turning_r`, `wz_max`를 확인한다.

#### `Map: /local_costmap/costmap`

| 항목 | 값 |
| :--- | :--- |
| Display type | `rviz_default_plugins/Map` |
| Topic | `/local_costmap/costmap` |
| Update Topic | `/local_costmap/costmap_updates` |
| Message type | `nav_msgs/OccupancyGrid` |
| 출처 | `controller_server` 내부 `local_costmap` |
| 의미 | MPPI가 비용 계산에 참고하는 로컬 costmap |

현재 설정:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 20
      height: 20
      resolution: 0.1
      plugins: ["inflation_layer"]
```

의미:

| 파라미터 | 의미 |
| :--- | :--- |
| `global_frame: odom` | costmap 좌표 기준 |
| `robot_base_frame: base_link` | 차량 위치 기준 |
| `rolling_window: true` | 차량 주변을 따라 창이 이동 |
| `width: 20`, `height: 20` | 20m x 20m 로컬 영역 |
| `resolution: 0.1` | 한 셀 10cm |

현재는 obstacle sensor layer 없이 `inflation_layer`만 켜져 있다. 따라서 초기 단계에서는 장애물 회피보다는 costmap frame, footprint, controller 연결 상태를 검증하는 용도에 가깝다.

정상 상태:

- `odom` 기준으로 차량 주변에 costmap 창이 보인다.
- 차량이 움직이면 rolling window가 따라온다.
- `/local_costmap/published_footprint`가 costmap 위에 겹쳐 보인다.

문제 징후:

- costmap이 안 보이면 `controller_server` lifecycle 상태, TF, `local_costmap` 활성화 상태를 확인한다.
- costmap이 차량과 따로 놀면 `global_frame`, `robot_base_frame`, TF를 확인한다.

#### `/local_costmap/costmap_updates`

| 항목 | 값 |
| :--- | :--- |
| RViz 위치 | Map display의 `Update Topic` |
| Topic | `/local_costmap/costmap_updates` |
| Message type | `map_msgs/OccupancyGridUpdate` |
| 출처 | `local_costmap` |
| 의미 | costmap 중 바뀐 영역만 전달하는 부분 업데이트 |

RViz의 `Map` display는 전체 지도인 `/local_costmap/costmap`과 부분 업데이트인 `/local_costmap/costmap_updates`를 함께 사용할 수 있다. 전체 costmap을 매번 보내는 대신 변경된 영역만 업데이트하면 통신량을 줄일 수 있다.

현재 YAML에는 다음 설정이 있다.

```yaml
always_send_full_costmap: true
```

그래서 초기 디버깅에서는 `/local_costmap/costmap` 중심으로 보면 된다. `/costmap_updates`는 RViz Map display가 업데이트 최적화를 위해 같이 들고 있는 토픽으로 이해하면 된다.

#### `Polygon: /local_costmap/published_footprint`

| 항목 | 값 |
| :--- | :--- |
| Display type | `rviz_default_plugins/Polygon` |
| Topic | `/local_costmap/published_footprint` |
| Message type | `geometry_msgs/PolygonStamped` |
| 출처 | `local_costmap` |
| 의미 | costmap에 투영된 차량 footprint |

현재 설정:

```yaml
footprint: "[[1.15, 0.5], [1.15, -0.5], [-0.35, -0.4], [-0.35, 0.4]]"
```

이는 `base_link` 기준의 임시 직사각형이다.

```text
x = +1.0  앞쪽
x = -1.0  뒤쪽
y = +0.5  좌측
y = -0.5  우측
```

정상 상태:

- footprint 중심/기준이 `base_link`와 맞아야 한다.
- 차량 heading이 바뀌면 polygon도 같이 회전해야 한다.
- costmap 위에서 차량이 차지하는 영역이 직관적으로 맞아야 한다.

나중에 실제 차량 제원에 맞춰 수정해야 할 항목:

- 차량 앞쪽 길이
- 차량 뒤쪽 길이
- 차량 폭
- `base_link`가 차량 중앙인지 후륜축인지
- 안전 여유 폭

#### RViz tools: `/initialpose`, `/clicked_point`, `/goal_pose`

`mppi.rviz`에는 RViz 기본 tool도 포함되어 있다.

| Tool | Topic | Type | 현재 MPPI에서의 역할 |
| :--- | :--- | :--- | :--- |
| `2D Goal Pose` | `/goal_pose` | `PoseStamped` | 목표점 클릭 입력으로 사용 |
| `2D Pose Estimate` | `/initialpose` | `PoseWithCovarianceStamped` | 현재 1차 MPPI 구조에서는 사용 안 함 |
| `Publish Point` | `/clicked_point` | `PointStamped` | 현재 1차 MPPI 구조에서는 사용 안 함 |

`/goal_pose`만 `mppi_path_client`가 실제로 사용한다. `/initialpose`와 `/clicked_point`는 RViz 기본 도구로 남아 있지만, 현재 MPPI bringup에서는 직접 소비하지 않는다.

#### 추천 확인 순서

RViz를 켠 뒤 아래 순서대로 보면 문제를 빠르게 좁힐 수 있다.

1. `Axes: odom`, `Axes: base_link`
   - TF가 정상인지 확인한다.

2. `EKF Encoder IMU Path`
   - `/odom/ekf_encoder_imu` 기반 실제 주행 추정이 자연스러운지 확인한다.

3. `Pose: /goal_pose`
   - RViz goal 클릭이 `odom` frame으로 발행되는지 확인한다.

4. `/mppi/active_path`
   - goal 또는 csv path가 MPPI 입력 path로 변환됐는지 확인한다.

5. `/transformed_global_plan`
   - MPPI 내부 path handler가 현재 볼 경로 조각을 만들었는지 확인한다.

6. `/trajectories`
   - MPPI 후보 궤적 rollout이 생성되는지 확인한다.

7. `/local_costmap/costmap`, `/local_costmap/published_footprint`
   - costmap과 차량 footprint가 `odom/base_link` 기준에 맞는지 확인한다.

#### 증상별 RViz 해석

| 증상 | RViz에서 볼 것 | 원인 후보 |
| :--- | :--- | :--- |
| 차량 축이 안 보임 | `Axes: base_link` | `odom -> base_link` TF 없음 |
| goal 클릭해도 path 없음 | `/goal_pose`, `/mppi/active_path` | RViz fixed frame 불일치, `mppi_path_client` 미실행 |
| active path는 있는데 MPPI가 안 움직임 | `/transformed_global_plan`, `/cmd_vel` | FollowPath action 문제, path pruning/TF 문제 |
| transformed plan이 비어 있음 | `/mppi/active_path`, `/transformed_global_plan` | path 시작점이 너무 멂, frame 불일치 |
| trajectories가 안 보임 | `/trajectories` | `visualize: false`, FollowPath goal 미전송 |
| trajectories가 이상하게 회전 | `/trajectories`, `base_link` axes | yaw 부호, `min_turning_r`, `wz_max` 문제 |
| costmap이 차량과 어긋남 | `/local_costmap/costmap`, footprint | `global_frame`, `robot_base_frame`, TF 문제 |
| footprint 크기가 안 맞음 | `/local_costmap/published_footprint` | `footprint` 파라미터가 실제 차량과 다름 |

---

## 11. 튜닝 순서

처음부터 고속으로 실행하지 않는다. 아래 순서로 하나씩 올린다.

### 11.1 속도 제약

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

### 11.2 계산량

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

### 11.3 경로 추종 성향

| 증상 | 조정 |
| :--- | :--- |
| 경로에서 멀어짐 | `PathAlignCritic.cost_weight` 증가 |
| 목표로 잘 안 감 | `PathFollowCritic.cost_weight` 증가 |
| 커브 진입이 늦음 | `PathAngleCritic.cost_weight` 증가 |
| 조향이 떨림 | `wz_std` 감소, `temperature` 증가 |
| 너무 공격적임 | `temperature` 증가 |
| 너무 둔함 | `temperature` 감소 |

### 11.4 costmap

초기에는 장애물 layer 없이 inflation layer만으로 MPPI 연결을 먼저 검증한다. 이후 LiDAR 또는 장애물 센서가 준비되면 `ObstacleLayer`를 추가한다.

장애물 layer 추가 시 확인할 것:

- 센서 topic type
- QoS 호환성
- sensor frame과 `base_link` 사이 TF
- `marking`, `clearing`
- obstacle height 범위
- `inflation_radius`
- 차량 footprint

---

## 12. 실패 증상별 점검

| 증상 | 원인 후보 | 점검 |
| :--- | :--- | :--- |
| `/follow_path` server 없음 | lifecycle inactive | `ros2 lifecycle get /controller_server` |
| MPPI가 명령을 안 냄 | path 미전송 또는 frame 불일치 | path `header.frame_id`, TF 확인 |
| RViz 클릭 후 움직이지 않음 | `/goal_pose` frame이 `odom`이 아님 | RViz Fixed Frame을 `odom`으로 변경 |
| CSV path가 거부됨 | path frame 불일치 또는 waypoint 부족 | `header.frame_id=odom`, pose 2개 이상 확인 |
| `Failed to transform` | TF 지연/누락 | `tf2_echo odom base_link`, `transform_tolerance` 증가 |
| 차량이 반대로 조향 | `angular.z` 또는 조향 변환 부호 오류 | 좌회전 시 부호 확인 |
| 제어 입력이 튐 | 오도메트리 불연속 또는 속도 노이즈 | `/odom/ekf_encoder_imu` twist 확인 |
| 경로를 지나쳤는데 되돌아감 | path pruning 부족 | `prune_distance` 증가 |
| CPU 과부하 | MPPI 샘플 수/시각화 과다 | `batch_size` 감소, `visualize: false` |
| 좁은 구간에서 떨림 | cost critic 과도 | `CostCritic.cost_weight`, inflation 조정 |

---

## 13. 단계별 구현 계획

### Phase 1. 로컬 오도메트리 안정화

목표:

- `/odom/ekf_encoder_imu`가 30 Hz 이상으로 안정 발행
- `odom -> base_link` TF 정상 발행
- `twist.linear.x`, `twist.angular.z` 부호 검증

완료 기준:

```bash
ros2 topic hz /odom/ekf_encoder_imu
ros2 run tf2_ros tf2_echo odom base_link
```

### Phase 2. Nav2 MPPI controller server 실행

목표:

- `src/navigation2` 소스 빌드
- `planner_server`, `controller_server` active 상태 진입
- `/compute_path_to_pose` action server 활성화
- `/follow_path` action server 활성화
- `/cmd_vel` topic 생성

완료 기준:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 action info /follow_path
ros2 action info /compute_path_to_pose
```

### Phase 3. RViz goal planner 경로 및 CSV FollowPath 성공

목표:

- RViz2 `2D Goal Pose` 클릭으로 `planner_server`가 path 생성
- `/mppi/csv_path` 또는 CSV 파일로 준비된 path 전송
- MPPI가 `/cmd_vel` 생성

완료 기준:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /mppi/active_path
```

### Phase 4. 저수준 제어 연결

목표:

- `/cmd_vel`을 차량 구동/조향 명령으로 변환
- 저속 직선 주행 성공
- 저속 곡선 주행 성공

완료 기준:

- 직선 path에서 큰 진동 없이 전진
- 곡선 path에서 조향 방향이 정상
- 목표 근처에서 정지 또는 goal success

### Phase 5. costmap과 장애물 회피 확장

목표:

- local costmap에 장애물 layer 추가
- MPPI `CostCritic`으로 장애물 비용 반영
- footprint와 inflation radius 튜닝

완료 기준:

- RViz에서 local costmap 장애물 표시
- 장애물 주변 후보 궤적 비용 증가
- `/cmd_vel`이 장애물 회피 방향으로 변화

### Phase 6. 전역 프레임 확장

목표:

- 필요 시 `map -> odom` TF 추가
- map frame path를 MPPI에 전달
- 장거리 경로 추종에서 누적 오차 보정

완료 기준:

```text
map -> odom -> base_link
```

TF tree가 유지되고, `map` frame path가 controller server 내부에서 local control frame으로 정상 변환된다.

---

## 14. 핵심 결론

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
