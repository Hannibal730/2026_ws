# utm_pkg

`utm_pkg` converts a recorded GNSS path to a local UTM/ENU CSV map and
visualizes a second F9P/F9R run in the REP-105 `map` frame. It also estimates
vehicle global yaw from the physical rear-to-front antenna vector.

Vehicle convention used here:

- ZED-F9P: rear antenna, `/f9p/fix`
- ZED-F9R: front antenna, `/f9r/fix`
- `/global_yaw`: F9P rear -> F9R front, radians, ROS ENU convention
- `/global_azimuth_deg`: north=0 degrees, clockwise positive

## Coordinate contract

The first stable samples of the mapping bag define the origin. CSV columns are:

```text
map_x = UTM easting - origin easting   (east, metres)
map_y = UTM northing - origin northing (north, metres)
map_z = altitude - origin altitude     (up, metres)
```

All visualization messages use `frame_id: map`. This package deliberately does
not publish `map -> odom`, `odom -> base_link`, or `map -> base_link`. In the
final REP-105 system, the Global EKF alone must own `map -> odom`, while the
Local EKF owns `odom -> base_link`.

## Build

```bash
cd ~/git_projects/Capstone-Project
sr
sv
colcon build --packages-select utm_pkg --symlink-install
si
```

The full commands work on a computer without the bash aliases:

```bash
cd ~/git_projects/Capstone-Project
source /opt/ros/humble/setup.bash
source .venv/bin/activate
colcon build --packages-select utm_pkg --symlink-install
source install/setup.bash
```

## 1. Convert the mapping bag

This keeps valid GNSS fixes for visualization and records RTK status and
covariance in every CSV row:

```bash
mkdir -p ~/utm_output
ros2 run utm_pkg bag_to_enu_csv -- \
  --bag ~/git_projects/gnss_260720_ws/src/gnss_path_visualizer/bags/260720_spec_1 \
  --topic /f9p/fix \
  --output ~/utm_output/260720_spec_1_f9p_map.csv
```

For a competition-quality surveyed map, record the route again until RTK FIXED
is continuous, then make a strict map:

```bash
ros2 run utm_pkg bag_to_enu_csv -- \
  --bag <mapping_bag> \
  --topic /f9p/fix \
  --output ~/utm_output/competition_map.csv \
  --fixed-only \
  --max-horizontal-stddev 0.20
```

The converter also writes `<output>.metadata.json`, containing the source bag,
origin, filters, status counts, rejection counts, path length, and segment
count. This makes a CSV result reproducible instead of leaving its origin and
filter settings unknown.

## 2. Visualize a bag and calculate global yaw

```bash
ros2 launch utm_pkg utm_visualization.launch.py \
  csv_file:=$HOME/utm_output/260720_spec_1_f9p_map.csv \
  bag_path:=$HOME/git_projects/gnss_260720_ws/src/gnss_path_visualizer/bags/260720_spec_2 \
  play_bag:=true
```

Set `loop_bag:=true` only when repeated playback is useful. The default is one
playback so a new bag starts from a clean node state.

The optional `playback_rate` launch argument changes only rosbag playback
speed. Its default is `1.0`, which replays the bag at the recorded time scale.

For live receivers, start both u-blox publishers and run:

```bash
ros2 launch utm_pkg utm_visualization.launch.py \
  csv_file:=$HOME/utm_output/competition_map.csv
```

## Outputs

```text
/utm/reference_path          nav_msgs/Path
/utm/reference_quality       visualization_msgs/Marker
/utm/f9p/rear                geometry_msgs/PointStamped
/utm/f9r/front               geometry_msgs/PointStamped
/utm/f9p/rear_path           nav_msgs/Path
/utm/f9r/front_path          nav_msgs/Path
/global_yaw                  std_msgs/Float64 (quality-gated ROS yaw, rad)
/global_yaw/raw              std_msgs/Float64 (diagnostic only)
/global_yaw/smoothed         std_msgs/Float64 (diagnostic/controller comparison)
/global_yaw/valid            std_msgs/Bool
/global_yaw/variance         std_msgs/Float64
/global_azimuth_deg          std_msgs/Float64
/utm/global_yaw_pose         geometry_msgs/PoseWithCovarianceStamped
/utm/diagnostics             diagnostic_msgs/DiagnosticArray
```

For a future Global EKF, use `/utm/global_yaw_pose` and configure that input to
fuse yaw only. It has a timestamp and covariance; the headerless Float64 topic
is retained for easy inspection and assignment compatibility.

## Quality protection

`/global_yaw` is published only when all checks pass:

1. Both antenna fixes are valid and RTK FIXED.
2. Both horizontal standard deviations are at most 0.25 m.
3. F9P is interpolated to the F9R timestamp, or the nearest sample is within
   the synchronization tolerance.
4. Rear-to-front baseline is 0.65 to 1.25 m.
5. The heading change is physically plausible (at most 120 deg/s).
6. If no accepted heading arrives for 0.5 s, `/global_yaw/valid` becomes false.

The supplied bags measured a FIXED baseline range of 0.72 to 1.18 m. Without
FIX filtering, corrupt pairs reached 3.53 m. The defaults therefore include
margin around measured good data while rejecting the observed failures.

## QoS

- GNSS subscriptions: Best Effort, Volatile, Keep Last 30.
- Global yaw, points, pose, validity, diagnostics: Reliable, Volatile, Keep
  Last 10. Local EKF/diagnostic subscribers should receive every accepted state.
- Static CSV path/quality: Reliable, Transient Local, Keep Last 1. RViz opened
  later still receives the map.
- Trails/markers: Reliable, Volatile, Keep Last 1. Trails are enabled by
  default, capped at 1000 points, and published at 1 Hz to keep the route
  visible without allowing RViz traffic to exhaust memory.

## RViz colors

- CSV quality: green=RTK FIXED segment, orange=non-fixed segment
- F9P rear track/point: blue
- F9R front track/point: red
- Physical antenna baseline: gray
- Accepted `/global_yaw`: yellow arrow
- Rejected pair: a gray candidate arrow follows the current rear-to-front
  antenna vector. `/global_yaw/valid` is false and the candidate is not
  published as a valid measurement.
- The on-screen status is arranged vertically as Global yaw, baseline, F9P,
  F9R, and output validity.

## Competition work still required

- Measure the antenna baseline rigidly after final installation and tighten the
  baseline limits around that value.
- Use u-blox `NavPVT` carrier-phase flags or `NavRELPOSNED9` validity/heading
  flags when those topics are recorded. `NavSatFix status=2` is fixed only
  because this repository's u-blox driver maps it that way.
- Record an entirely RTK-FIXED mapping lap; do not use orange portions as the
  final MPPI reference path.
- Calibrate the lever arm from the rear antenna to `base_link` and apply it in
  the localization configuration.
- Fuse dual-GNSS yaw in the Global EKF with GNSS position. Keep IMU+encoder in
  the Local EKF, then validate one owner each for `map -> odom` and
  `odom -> base_link`.
- Compare GNSS and EKF timestamps/latency under full CPU/network load and record
  rosbag topics containing fix-state diagnostics, not only NavSatFix.
- Verify the receiver's achievable RTK rate with the final constellation setup.
  The ZED-F9P-04B data sheet lists a 7 Hz maximum RTK navigation rate when
  GPS+GLONASS+Galileo+BeiDou are all enabled, and 15 Hz for GPS+Galileo. Do not
  reduce constellations merely to raise Hz until open-course FIX availability
  and accuracy have been compared.
- Add track cleanup (outlier review, lap closure, resampling, curvature and
  direction checks) before feeding the CSV to MPPI.
