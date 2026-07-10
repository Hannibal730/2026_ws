# clone 하고 초기 설정
```
cd /home/$(whoami)/Mando2026_ws
python3 -m venv .venv
source .venv/bin/activate
pip3 install -r requirements.txt
sudo apt update
sudo apt install -y libasio-dev ros-humble-diagnostic-updater ros-humble-nmea-msgs
```

# 기본 준비
```
cd /home/$(whoami)/Mando2026_ws
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
sudo chmod 666 /dev/tty*
```

# Publisher
## `/encoder/*`
```
ros2 run encoder_pkg encoder_publisher --ros-args \
  --params-file src/encoder_pkg/config/encoder_params.yaml \
  -p serial_port:=/dev/ttyACM0 \
  -p baudrate:=115200
```

## `/imu/*`
```
ros2 run imu_pkg imu_publisher --ros-args \
  --params-file src/imu_pkg/config/imu_params.yaml \
  -p serial_port:=/dev/ttyUSB0 \
  -p baudrate:=115200
```

## `/odom/encoder*`
```
ros2 run odom_pkg encoder_odometry --ros-args \
  --params-file src/odom_pkg/config/odom_params.yaml \
  -p publish_tf:=false
```

## `/odom/imu*`
IMU 단독 dead-reckoning (gyro z yaw 적분 + 전진 가속도 적분).
시작 시 센서를 완전히 정지시킨 채 캘리브레이션(기본 5초)이 진행된다.
```
ros2 run odom_pkg imu_odometry --ros-args \
  --params-file src/odom_pkg/config/odom_params.yaml \
  -p publish_tf:=false
```

## `/odom/encoder_imu*`
raw dead-reckoning (엔코더+IMU 단순 적분).
```
ros2 run odom_pkg encoder_imu_odometry --ros-args \
  --params-file src/odom_pkg/config/odom_params.yaml \
  -p publish_tf:=false
```

## `/odom/ekf_encoder_imu*`
EKF는 `/odom/encoder` + `/imu/data`를 융합한다. 이 노드는 `encoder_odometry`를 **같은 프로세스에서 내부 실행**(`publish_tf:=false`)하여 `/odom/encoder`를 직접 발행하고, `ekf_node`를 띄운 뒤 필터 결과 path를 만든다. 따라서 별도로 `encoder_odometry`를 실행할 필요는 없고, `/imu/data`(위 `/imu/*` 항목)만 미리 켜져 있으면 된다.
```
ros2 run odom_pkg ekf_encoder_imu_odometry
```


<!-- ## `/f9p/*`, `/ublox_gps_node/*`
```
ros2 launch ublox_gps ublox_f9p_launch.py serial_port:=/dev/ttyUSB0 baudrate:=115200
```

## `/f9r/*`, `/ublox_gps_node/*`
```
ros2 launch ublox_gps ublox_f9r_launch.py serial_port:=/dev/ttyUSB1 baudrate:=115200 -->
```

# ROS BAG 만들기
```
ros2 bag record -e "(/imu/.*|/encoder/.*)"
```

---
udev 작업 필요
