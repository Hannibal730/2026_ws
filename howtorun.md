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

만일 Workspace가 `/home/dev` 안에 있을 때:
```
cd /home/$(whoami)/dev/Mando2026_ws
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
sudo chmod 666 /dev/tty*
```


# Publisher
## `/encoder/*`
```
ros2 run encoder_pkg encoder_publisher --ros-args -p serial_port:=/dev/ttyACM0 -p baudrate:=115200
```

## `/imu/*`
```
ros2 run imu_pkg imu_publisher --ros-args -p serial_port:=/dev/ttyUSB0 -p baudrate:=115200
```

## `/odom`, `/odom_path`, `/tf`
```
ros2 run odom_pkg encoder_imu_odometry --ros-args --params-file src/odom_pkg/config/odom_params.yaml
```

## `/f9p/*`, `/ublox_gps_node/*`
```
ros2 launch ublox_gps ublox_f9p_launch.py serial_port:=/dev/ttyUSB0 baudrate:=115200
```

## `/f9r/*`, `/ublox_gps_node/*`
```
ros2 launch ublox_gps ublox_f9r_launch.py serial_port:=/dev/ttyUSB1 baudrate:=115200
```

# RVIZ
## RVIZ만 실행
```
rviz2 -d install/rviz_pkg/share/rviz_pkg/config/imu_odometry.rviz
```
## `/encoder/*`, `/imu/*`, `/odom`도 같이 실행
```
ros2 launch rviz_pkg encoder_imu_rviz.launch.py imu_serial_port:=/dev/ttyUSB0 imu_baudrate:=115200 encoder_serial_port:=/dev/ttyACM0 encoder_baudrate:=115200
```

---
udev 작업 필요
