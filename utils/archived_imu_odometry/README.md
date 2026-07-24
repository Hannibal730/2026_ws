# Archived IMU Odometry

This folder keeps the old `imu_odometry` node outside of `odom_pkg`.

It was an IMU-only dead-reckoning/debug node that published `/odom/imu` and
`/odom/imu/path`. The current EKF flow uses `/imu/data` directly, so this node is
not part of the active odometry pipeline.
