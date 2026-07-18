# gnss_pkg

`gnss_pkg`는 ROS 2 Humble 환경에서 u-blox GNSS 수신기와 NTRIP 보정 데이터를 함께 사용하기 위한 패키지 모음입니다.

일반 GNSS 수신기는 위성 신호만으로 위치를 계산하므로 주변 환경, 대기 오차, 위성 궤도 오차 등에 의해 수 m 수준의 오차가 발생할 수 있습니다. RTK(Real-Time Kinematic)를 사용하면 기준국에서 생성한 RTCM 보정 데이터를 수신기에 전달하여 더 높은 정확도의 위치를 얻을 수 있습니다. 이 패키지는 그 과정을 ROS 2 토픽으로 연결합니다.

## 전체 동작 흐름

이 패키지의 핵심 흐름은 다음과 같습니다.

```text
u-blox GNSS receiver
  -> ublox_gps node
  -> /f9p/fix or /f9r/fix
  -> fix2nmea node
  -> /ntrip_client/nmea
  -> NTRIP caster
  -> /ntrip_client/rtcm
  -> ublox_gps node
  -> u-blox GNSS receiver
```

각 단계의 의미는 아래와 같습니다.

1. `ublox_gps` 노드는 u-blox 수신기에서 현재 GNSS 위치를 읽어 ROS 2 `NavSatFix` 메시지로 publish합니다.
2. `fix2nmea` 노드는 `NavSatFix` 메시지를 NTRIP 서버가 요구하는 NMEA GGA 문장으로 변환합니다.
3. `ntrip_client` 노드는 NMEA 문장을 NTRIP caster에 보내고, caster로부터 RTCM 보정 데이터를 받아옵니다.
4. `ntrip_client`가 publish한 RTCM 메시지는 다시 `ublox_gps` 노드로 전달됩니다.
5. `ublox_gps` 노드는 RTCM 보정 데이터를 실제 u-blox 수신기에 써서 RTK 보정이 적용되도록 합니다.

즉, GNSS 수신기의 대략적인 현재 위치를 NTRIP 서버에 알려주고, NTRIP 서버가 해당 위치에 맞는 보정 데이터를 보내주면, 그 보정 데이터를 다시 GNSS 수신기에 넣어 위치 정확도를 높이는 구조입니다.

## 포함된 주요 패키지

### 1. u-blox Driver

u-blox 드라이버는 KumarRobotics의 `ublox` ROS 패키지를 기반으로 합니다.

원본 저장소: <https://github.com/KumarRobotics/ublox.git>

이 드라이버는 u-blox 수신기와 시리얼 통신을 수행하고, 수신기에서 나오는 UBX 메시지를 ROS 2 메시지로 변환합니다. 이 워크스페이스에서는 주로 ZED-F9P 또는 ZED-F9R 수신기를 rover로 사용하는 구성을 제공합니다.

주요 파일은 다음과 같습니다.

- `ublox_gps/launch/ublox_f9p_launch.py`: ZED-F9P 실행용 launch 파일
- `ublox_gps/launch/ublox_f9r_launch.py`: ZED-F9R 실행용 launch 파일
- `ublox_gps/config/zed_f9p.yaml`: ZED-F9P 파라미터 설정
- `ublox_gps/config/zed_f9r.yaml`: ZED-F9R 파라미터 설정

기본 실행 명령은 다음과 같습니다.

```bash
ros2 launch ublox_gps ublox_f9p_launch.py
```

또는 ZED-F9R을 사용할 경우:

```bash
ros2 launch ublox_gps ublox_f9r_launch.py
```

기본 시리얼 포트는 launch 파일에 다음처럼 설정되어 있습니다.

- ZED-F9P: `/dev/ttyACM1`
- ZED-F9R: `/dev/ttyACM0`

장치 포트가 다르면 launch 실행 시 직접 지정할 수 있습니다.

```bash
ros2 launch ublox_gps ublox_f9p_launch.py serial_port:=/dev/ttyACM0 baudrate:=115200
```

u-blox 노드는 RTCM 보정 데이터를 받기 위해 `/ntrip_client/rtcm` 토픽을 구독합니다. 이 토픽으로 들어온 `rtcm_msgs/msg/Message` 데이터는 `gps_->sendRtcm(...)`을 통해 실제 수신기로 전달됩니다.

### 2. NTRIP Client

NTRIP Client는 LORD-MicroStrain의 ROS 2 패키지를 기반으로 합니다.

원본 저장소: <https://github.com/LORD-MicroStrain/ntrip_client.git>

NTRIP은 인터넷을 통해 기준국 보정 데이터(RTCM)를 받아오기 위한 프로토콜입니다. 여기서 `ntrip_client` 노드는 NTRIP caster에 접속하여 RTCM 데이터를 받아오고, ROS 2 토픽으로 publish합니다.

주요 파일은 다음과 같습니다.

- `ntrip_client/launch/ntrip_client_launch.py`: NTRIP 접속 정보와 노드 실행 설정
- `ntrip_client/scripts/ntrip_ros.py`: ROS 2 노드 구현
- `ntrip_client/src/ntrip_client/ntrip_client.py`: 실제 NTRIP 서버 접속 로직

기본 실행 명령은 다음과 같습니다.

```bash
ros2 launch ntrip_client ntrip_client_launch.py
```

launch 파일에서 설정하는 주요 값은 다음과 같습니다.

- `host`: NTRIP caster 주소
- `port`: NTRIP caster 포트, 일반적으로 `2101`
- `mountpoint`: 사용할 기준국 또는 보정 스트림 이름
- `authenticate`: 계정 인증 사용 여부
- `username`: NTRIP 계정 이름
- `password`: NTRIP 계정 비밀번호
- `rtcm_message_package`: RTCM 메시지 타입 선택, 이 패키지에서는 `rtcm_msgs` 사용

현재 launch 파일은 기본값으로 `www.gnssdata.or.kr`, `2101`, `SOUL-RTCM32`를 사용하도록 되어 있습니다. 다른 기준국이나 계정을 사용하려면 launch 파일을 수정하거나 실행 시 인자로 넘기면 됩니다.

예시:

```bash
ros2 launch ntrip_client ntrip_client_launch.py \
  host:=www.gnssdata.or.kr \
  port:=2101 \
  mountpoint:=SOUL-RTCM32 \
  authenticate:=True \
  username:=YOUR_USERNAME \
  password:=YOUR_PASSWORD \
  rtcm_message_package:=rtcm_msgs
```

주의할 점은 계정 정보입니다. `username`, `password`를 소스 코드나 공개 저장소에 그대로 남기면 보안 문제가 생길 수 있으므로, 실제 프로젝트에서는 환경 변수나 별도 설정 파일로 분리하는 것이 좋습니다.

`ntrip_client` 노드의 주요 토픽은 다음과 같습니다.

- Subscribe: `/ntrip_client/nmea`
- Publish: `/ntrip_client/rtcm`

`/ntrip_client/nmea`는 NTRIP caster에 보낼 현재 위치 NMEA 문장입니다. 일부 caster는 rover의 대략적인 위치를 알아야 주변 기준국에 맞는 보정 데이터를 제공합니다. `/ntrip_client/rtcm`은 caster에서 받은 RTCM 보정 데이터이며, u-blox 드라이버가 이 토픽을 구독합니다.

### 3. /fix to /nmea Parser

`fix2nmea`는 `sensor_msgs/msg/NavSatFix` 형식의 GNSS 위치 메시지를 `nmea_msgs/msg/Sentence` 형식의 NMEA GGA 문장으로 바꾸는 작은 변환 노드입니다.

NTRIP caster가 rover 위치를 요구하는 경우, 단순히 RTCM만 받는 것이 아니라 rover의 현재 위치를 NMEA 형태로 서버에 계속 보내야 합니다. 그런데 u-blox 드라이버에서 ROS 쪽으로 나오는 대표 위치 메시지는 `/fix` 계열의 `NavSatFix` 메시지입니다. 그래서 중간에 `fix2nmea`가 필요합니다.

실행 명령은 다음과 같습니다.

```bash
ros2 run fix2nmea fix2nmea
```

현재 `fix2nmea` 구현 기준 토픽은 다음과 같습니다.

- Subscribe: `/ublox_gps_node/fix`
- Publish: `/ntrip_client/nmea`

단, 제공된 u-blox launch 파일은 `/ublox_gps_node/fix`를 아래 토픽으로 remap합니다.

- ZED-F9P launch: `/f9p/fix`
- ZED-F9R launch: `/f9r/fix`

따라서 제공된 launch 파일을 그대로 사용하면 `fix2nmea`가 `/ublox_gps_node/fix`를 기다리지만 실제 위치는 `/f9p/fix` 또는 `/f9r/fix`로 나올 수 있습니다. 이 경우 `fix2nmea` 실행 시 remap을 추가해야 합니다.

ZED-F9P 예시:

```bash
ros2 run fix2nmea fix2nmea --ros-args -r /ublox_gps_node/fix:=/f9p/fix
```

ZED-F9R 예시:

```bash
ros2 run fix2nmea fix2nmea --ros-args -r /ublox_gps_node/fix:=/f9r/fix
```

## 권장 실행 순서

각 노드는 계속 실행되어야 하므로 보통 터미널을 3개 열어 실행합니다. 모든 터미널에서 먼저 워크스페이스 환경을 source해야 합니다.

```bash
cd ~/Mando2026_ws
source install/setup.bash
```

터미널 1: u-blox 수신기 실행

```bash
ros2 launch ublox_gps ublox_f9p_launch.py
```

터미널 2: `/fix`를 NMEA로 변환

```bash
ros2 run fix2nmea fix2nmea --ros-args -r /ublox_gps_node/fix:=/f9p/fix
```

터미널 3: NTRIP client 실행

```bash
ros2 launch ntrip_client ntrip_client_launch.py
```

ZED-F9R을 사용할 경우 위 명령에서 `ublox_f9p_launch.py`를 `ublox_f9r_launch.py`로 바꾸고, `/f9p/fix`를 `/f9r/fix`로 바꾸면 됩니다.

## 빌드 방법

처음 사용하거나 코드를 수정한 뒤에는 워크스페이스 루트에서 빌드합니다.

```bash
cd ~/Mando2026_ws
colcon build
source install/setup.bash
```

특정 패키지만 다시 빌드하고 싶다면 다음처럼 실행할 수 있습니다.

```bash
colcon build --packages-select fix2nmea
source install/setup.bash
```

## 토픽 확인 방법

노드가 제대로 연결되었는지 확인하려면 아래 명령을 사용할 수 있습니다.

전체 토픽 목록:

```bash
ros2 topic list
```

u-blox 위치 출력 확인:

```bash
ros2 topic echo /f9p/fix
```

NMEA 변환 결과 확인:

```bash
ros2 topic echo /ntrip_client/nmea
```

RTCM 보정 데이터 수신 확인:

```bash
ros2 topic echo /ntrip_client/rtcm
```

토픽 연결 관계를 확인하려면 다음 명령이 유용합니다.

```bash
ros2 node info /ublox_gps_node
ros2 node info /fix2nmea
ros2 node info /ntrip_client/ntrip_client
```

## 자주 확인해야 할 설정

### 시리얼 포트

u-blox 수신기가 어떤 포트로 잡혔는지 확인해야 합니다.

```bash
ls /dev/ttyACM*
```

launch 파일의 기본값과 실제 장치 포트가 다르면 `serial_port:=...` 인자로 맞춰야 합니다.

### baudrate

현재 launch 파일과 yaml 설정은 기본적으로 `115200` bps를 사용합니다. 수신기 설정이 다르면 통신이 되지 않으므로 수신기의 실제 baudrate와 맞춰야 합니다.

### dynamic_model

`zed_f9p.yaml`, `zed_f9r.yaml`에는 `dynamic_model: automotive`가 설정되어 있습니다. 차량 환경에서는 적절하지만, 고정 기준국처럼 움직이지 않는 장비라면 `stationary`가 더 적절할 수 있습니다.

### NTRIP mountpoint

`mountpoint`는 어떤 기준국 또는 보정 스트림을 사용할지 결정합니다. 잘못된 mountpoint를 사용하면 접속은 되더라도 RTCM이 오지 않거나, 현재 위치와 맞지 않는 보정 데이터가 들어올 수 있습니다.

## 문제 해결

### `/ntrip_client/rtcm`이 나오지 않는 경우

- NTRIP `host`, `port`, `mountpoint`가 맞는지 확인합니다.
- 인증이 필요한 caster라면 `authenticate`, `username`, `password`를 확인합니다.
- `/ntrip_client/nmea`가 publish되고 있는지 확인합니다.
- 인터넷 연결과 방화벽 설정을 확인합니다.

### `/ntrip_client/nmea`가 나오지 않는 경우

- `fix2nmea` 노드가 실행 중인지 확인합니다.
- u-blox 위치 토픽(`/f9p/fix` 또는 `/f9r/fix`)이 실제로 publish되는지 확인합니다.
- `fix2nmea` 실행 시 `/ublox_gps_node/fix` remap을 올바르게 줬는지 확인합니다.

### u-blox 위치 토픽이 나오지 않는 경우

- 수신기가 USB 또는 시리얼로 연결되어 있는지 확인합니다.
- launch 파일의 `serial_port`가 실제 포트와 일치하는지 확인합니다.
- 현재 사용자에게 시리얼 포트 접근 권한이 있는지 확인합니다.
- 안테나가 하늘이 보이는 위치에 있는지 확인합니다.

### RTK 상태가 되지 않는 경우

- `/ntrip_client/rtcm`에 데이터가 계속 들어오는지 확인합니다.
- u-blox 노드가 `/ntrip_client/rtcm`을 구독하고 있는지 확인합니다.
- NTRIP mountpoint가 현재 지역과 맞는지 확인합니다.
- 수신기와 안테나가 RTK를 지원하는 모델인지 확인합니다.
- 하늘 시야가 좋지 않거나 multipath가 심한 환경에서는 RTK fix가 늦거나 불안정할 수 있습니다.

## 핵심 요약

- `ublox_gps`: u-blox 수신기와 통신하고 GNSS 위치를 ROS 토픽으로 publish합니다.
- `fix2nmea`: ROS `NavSatFix` 위치를 NTRIP 서버용 NMEA GGA 문장으로 변환합니다.
- `ntrip_client`: NTRIP caster에서 RTCM 보정 데이터를 받아 `/ntrip_client/rtcm`으로 publish합니다.
- `/ntrip_client/rtcm`: u-blox 수신기에 들어가는 RTK 보정 데이터입니다.
- `/ntrip_client/nmea`: NTRIP caster에 rover의 현재 위치를 알려주기 위한 NMEA 문장입니다.
