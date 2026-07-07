# Mandol_ws
---
## 인지 팀
#### GPS: 최대승, 유승민, 이현수
#### Camera: 이승민
---
## 판단 팀
#### 유승민
---
## 제어 팀
#### 박준성, 류승훈
---

## Journey to start

#### gps open source setting

```bash
sudo apt update
sudo apt install ros-humble-rtcm-msgs ros-humble-nmea-msgs
sudo apt install ros-humble-tf-transformations
sudo apt install ros-humble-ackermann-msgs ros-humble-nav2-bringup
```

#### f9p, f9r

https://github.com/olvdhrm/RTK_GPS_NTRIP


#### ntrip
https://github.com/SGroe/ntrip_client_ros2


ros2 bag  play  100_bag/ -l

colcon build --packages-select gps_to_utm



---
* f9r이 만든 백파일에서 /f9r/fix 정보를 사용하여 utm 좌표값이 담긴 csv 파일 생성 모듈
# Mandol_ws/src/gps_to_utm/src/f9r_to_csv.py
ros2 run gps_to_utm f9r_to_csv

* f9r, f9p hz 확인
ros2 topic hz /f9p/fix
ros2 topic hz /f9r/fix

* 모든 차선 시각화
visualize_all_csv.py
---

1. USB 포트에 f9r, f9p, 아두이노 케이블 모두 꽂은 후에 아래 코드 입력하기
ls /dev/ttyACM* /dev/ttyUSB*
이후 f9r.yaml, f9r.yaml, serial_bridge.py의 ttyACM 번호 수정하기.ㄴ

/home/hannibal/Mandol_ws/src/RTK_GPS_NTRIP/ntrip_client/launch/ntrip_client_launch.py 에서 DeclareLaunchArgument('mountpoint', default_value='SUWN-RTCM31'),  수정  SONP-RTCM31

------------------------------------------------

1. 아두이노만 꽂기
2. ls /dev/ttyACM* /dev/ttyUSB* 으로 0인지 확인
3. 모든 노드 끈 채로 아두이노 업로드. 만약 안 된다면 시리얼 브릿지 노드를 켰다가 끄고 업로드 재시도. 업로드 성공한 이후에 시리얼 브릿지 노드 켜기
4. ros2 run serial_bridge serial_bridge
5. f9p 꽂기
6. ls /dev/ttyACM* /dev/ttyUSB* 으로 1인지 확인
7. ros2 launch ublox_gps ublox_f9p_launch.py
8. f9r 꽂기
9.  ls /dev/ttyACM* /dev/ttyUSB* 으로 2인지 확인
10. ros2 launch ublox_gps ublox_f9r_launch.py
11. 나머지 노드들 실행

RTK 마운트포인트 수정 필요.
/home/hannibal/Mandol_ws/src/RTK_GPS_NTRIP/ntrip_client/launch/ntrip_client_launch.py 에서 DeclareLaunchArgument('mountpoint', default_value='SUWN-RTCM31'),  수정  SONP-RTCM31

두 곳에서 맵 수정 필요.
/home/hannibal/Mandol_ws/src/gps_to_utm/config/tf_gps_csv.yaml
/home/hannibal/Mandol_ws/src/path_planning/config/csv_detector.yaml

-----------------------------------------------

# 1_T_right_P_right
# 2_T_right_P_left
# 3_T_left_P_right
# 4_T_left_P_left


1. 백파일 폴더 경로에서 백파일 재생
# hannibal@hannibal:~/Mandol_ws/rosbag/gps_bag_9_16$
ros2 bag play T_parallel_2 --topics /f9p/fix /f9r/fix -l
hannibal@hannibal:~/Mandol_ws/rosbag/gps_bag_9_20$ ros2 bag play rosbag2_2025_09_20-15_57_24 --topics /f9r/fix -l
hannibal@hannibal:~/Mandol_ws/rosbag/gps_bag_9_16$ ros2 bag play T_parallel_3 -l

1. 실제 f9r, f9p 작동
# Mandol_ws/src/RTK_GPS_NTRIP/ublox_gps/launch
ros2 launch ublox_gps ublox_f9p_launch.py
ros2 launch ublox_gps ublox_f9r_launch.py
ros2 run fix2nmea fix2nmea
ros2 launch ntrip_client ntrip_client_launch.py

---

2. f9r이 생성하는 /f9r/fix 정보를 섭하여 /f9r_utm 실시간 발행 노드
# Mandol_ws/src/gps_to_utm/src/f9p_to_utm.cpp
ros2 run gps_to_utm f9p_to_utm

3. f9p가 생성하는 /f9p/fix 정보를 섭하여 /f9p_utm 실시간 발행 노드
# Mandol_ws/src/gps_to_utm/src/f9p_to_utm.cpp
ros2 run gps_to_utm f9r_to_utm

4. f9p(전륜축), f9r(후륜축) 센서로 /azimuth_angle 실시간 발행 노드
# Mandol_ws/src/gps_to_utm/src/azimuth_angle_calculator.cpp
ros2 run gps_to_utm azimuth_angle_calculator_node

5. csv를 /csv_path로 발행하고, /csv_path랑 gps를 tf하고, rviz2에서 시각화하는 노드 (csv path 수정 필요)
# Mandol_ws/src/gps_to_utm/src/tf_gps_csv.cpp
ros2 run gps_to_utm tf_gps_csv_node
또는 config를 수정하고 재빌드 없이 아래 런치 파일로
ros2 launch gps_to_utm tf_gps_csv.launch.py


위 2~5를 하나의 런치파일로 묶음.
# Mandol_ws/src/gps_to_utm/launch/total.launch.py
ros2 launch gps_to_utm total.launch.py

---
아래 4가지 노드는 주행을 새로 시작할 때마다 Ctrl+C 이후 새로 켠 이후에 gps 센서 작동시키기.
---

6. f9r의 roi_path와 roi_end 를 발행하는 노드
# Mandol_ws/src/path_planning/src/f9r_roi_path.cpp
ros2 run path_planning f9r_roi_path
또는 config를 수정하고 재빌드 없이 아래 런치 파일로
ros2 launch path_planning f9r_roi_path.launch.py

6. f9r의 front_roi_path를 사용하여 퓨어퍼슛하는 노드
# Mandol_ws/src/path_planning/src/f9r_roi_path.cpp
ros2 run path_planning pure_pursuit_node
또는 config를 수정하고 재빌드 없이 아래 런치 파일로
ros2 launch path_planning pure_pursuit.launch.py

6. UTM 좌표계 상에서 미션스테이트 분기영역을 지정하고, f9r이 들어가면 분기 토픽 발행하는 노드 (csv path 수정 필요)
# Mandol_ws/src/path_planning/src/csv_radius_detector.cpp
ros2 run path_planning csv_radius_detector_node
또는 config를 수정하고 재빌드 없이 아래 런치 파일로
ros2 launch path_planning csv_detector.launch.py

6. 정지구간 -> T자 주차 -> 평행 주차 단계에 따라서 미션 수행 토픽 발행하는 코드
# Mandol_ws/src/mission_supervisor/mission_supervisor/mission_supervisor.py
ros2 run mission_supervisor mission_supervisor_node

---
승민이형 코드

1.
sudo udevadm trigger 

ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/camera2 --params-file /home/hannibal/Mandol_ws/src/usb_cam/config/params_2.yaml

2.
ros2 launch realsense2_camera my_realsense.launch.py

3.
ros2 run mando_vision vision_node

4.
ros2 run mando_vision unified_recorder

---






ros2 bag record -a

ros2 bag record /robot/scan /robot/odom

9월20일 연습주행 빽파일 (f9r 하나만으로 딴 용인백)
hannibal@hannibal:~/gps_bag_9_20$ ros2 bag play rosbag2_2025_09_20-15_57_24