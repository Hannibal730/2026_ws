# utm_pkg 실행 및 공유 방법

이 문서는 ROS 2 Humble 환경에서 다음 작업을 재현하기 위한 실행 안내서다.

1. GNSS rosbag의 `NavSatFix`를 UTM 기반 로컬 ENU CSV로 변환한다.
2. CSV 기준 경로와 F9P/F9R 위치를 REP-105 `map` 프레임에 표시한다.
3. F9P 후방에서 F9R 전방으로 향하는 Global yaw를 계산하고 품질을 검사한다.

## 차량 및 좌표 규칙

```text
F9P = 후방 안테나 = /f9p/fix
F9R = 전방 안테나 = /f9r/fix
Global yaw = F9P rear -> F9R front
map X = East
map Y = North
map Z = Up
```

`/global_yaw`는 ROS ENU yaw이므로 동쪽이 0도이고 반시계 방향이 양수다.
북쪽 0도, 시계 방향 양수인 값은 `/global_azimuth_deg`에서 확인한다.

## 공유해야 할 파일

다른 컴퓨터에 전달할 때 다음 폴더를 통째로 전달한다.

```text
src/utm_pkg/
```

데이터 재현이 필요하면 다음 파일도 함께 전달한다.

```text
260720_spec_1/                         # 기준 CSV 생성용 rosbag
260720_spec_2/                         # F9P/F9R 비교용 rosbag
260720_spec_1_f9p_map.csv              # 이미 생성한 기준 경로
260720_spec_1_f9p_map.csv.metadata.json
```

`build/`, `install/`, `log/`, `.venv/`, `__pycache__/`는 다른 컴퓨터로
전달하지 않는다. 각 컴퓨터에서 다시 빌드한다.

## 다른 컴퓨터 최초 설정

`<workspace>`는 실제 ROS 2 워크스페이스 경로로 바꾼다.

```bash
mkdir -p <workspace>/src
cp -r utm_pkg <workspace>/src/
cd <workspace>

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-rosbag2 \
  ros-humble-rviz2

python3 -m venv .venv --system-site-packages
source /opt/ros/humble/setup.bash
source .venv/bin/activate
colcon build --packages-select utm_pkg --symlink-install
source install/setup.bash
```

설치 확인:

```bash
ros2 pkg executables utm_pkg
```

정상 출력:

```text
utm_pkg bag_to_enu_csv
utm_pkg dual_gnss_map
```

## 1. spec_1 rosbag을 ENU CSV로 변환

먼저 bag 안의 GNSS 토픽을 확인한다.

```bash
ros2 bag info <spec_1_bag_path>
```

과제 데이터의 품질 상태까지 보존하는 CSV:

```bash
cd <workspace>
sr
sv
si

mkdir -p ~/utm_output
ros2 run utm_pkg bag_to_enu_csv -- \
  --bag <spec_1_bag_path> \
  --topic /f9p/fix \
  --output ~/utm_output/260720_spec_1_f9p_map.csv
```

만도대회용 기준 경로는 RTK FIXED 데이터로 다시 기록한 뒤 엄격하게 생성한다.

```bash
ros2 run utm_pkg bag_to_enu_csv -- \
  --bag <competition_mapping_bag_path> \
  --topic /f9p/fix \
  --output ~/utm_output/competition_map.csv \
  --fixed-only \
  --max-horizontal-stddev 0.20
```

생성 결과:

```text
~/utm_output/260720_spec_1_f9p_map.csv
~/utm_output/260720_spec_1_f9p_map.csv.metadata.json
```

## 2. 권장 실행: launch 한 번으로 재생

노드, RViz2, rosbag을 한 터미널에서 시작한다.

```bash
cd <workspace>
sr
sv
si

ros2 launch utm_pkg utm_visualization.launch.py \
  csv_file:=$HOME/utm_output/260720_spec_1_f9p_map.csv \
  bag_path:=<spec_2_bag_path> \
  play_bag:=true
```

`playback_rate`의 기본값은 `1.0`이며 rosbag을 원본 시간 기준으로 재생한다.
이 값은 실시간 센서의 발행 Hz 설정이 아니다.
`loop_bag` 기본값은 `false`다.

## 3. 디버깅 실행: 터미널 3개

### T1: UTM 및 Global yaw 노드

```bash
cd <workspace>
sr
sv
si
ros2 run utm_pkg dual_gnss_map --ros-args \
  --params-file src/utm_pkg/config/utm_params.yaml \
  -p csv_file:=$HOME/utm_output/260720_spec_1_f9p_map.csv \
  -p publish_sensor_trails:=true
```

### T2: RViz2

```bash
cd <workspace>
sr
sv
si
ros2 run rviz2 rviz2 -d src/utm_pkg/rviz/utm_visualization.rviz
```

### T3: spec_2 rosbag

```bash
cd <workspace>
sr
sv
si
ros2 bag play <spec_2_bag_path> \
  --rate 1.0 \
  --topics /f9p/fix /f9r/fix
```

## 4. 실시간 GNSS 실행

USB 번호는 연결 순서에 따라 달라질 수 있으므로 먼저 확인한다.

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

F9P와 F9R publisher를 각각 실행한 뒤 bag 재생 없이 시각화한다.

```bash
ros2 launch utm_pkg utm_visualization.launch.py \
  csv_file:=$HOME/utm_output/competition_map.csv
```

필수 입력 토픽:

```text
/f9p/fix
/f9r/fix
```

## 5. 상태 확인

```bash
ros2 topic echo /global_yaw/valid
ros2 topic echo /global_yaw --once
ros2 topic echo /global_yaw/raw --once
ros2 topic echo /utm/diagnostics --once
ros2 topic hz /f9p/fix
ros2 topic hz /f9r/fix
ros2 topic hz /global_yaw
```

RViz 색상:

```text
초록 CSV 선 = RTK FIXED 기준 경로
주황 CSV 선 = NON-FIXED 구간
파랑 = F9P 후방 궤적
빨강 = F9R 전방 궤적
회색 = 현재 F9P -> F9R baseline
노랑 화살표 = 유효 Global yaw
회색 화살표 = 품질 검사에서 거부된 현재 후보 yaw
```

## 문제 해결

패키지를 찾을 수 없을 때:

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select utm_pkg --symlink-install
source install/setup.bash
ros2 pkg executables utm_pkg
```

CSV 파일 오류가 발생하면 `csv_file`이 실제 파일의 절대경로인지 확인한다.

화살표가 주황색이면 화면의 거부 원인을 확인한다. `NON-FIXED`, covariance,
timestamp 동기화, baseline 범위 또는 yaw 변화율 검사에서 거부된 상태다.

다른 rosbag을 재생하기 전에는 기존 노드와 RViz를 `Ctrl+C`로 종료하고 다시
실행한다. 이전 궤적과 새 rosbag의 궤적이 섞이는 것을 방지하기 위해서다.
