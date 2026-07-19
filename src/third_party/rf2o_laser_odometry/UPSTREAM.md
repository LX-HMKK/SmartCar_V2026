# Upstream provenance

- Repository: https://github.com/Adlink-ROS/rf2o_laser_odometry
- Branch: `humble-devel`
- Commit: `313bb4c4123bcc0cc2e042f278312b19a3c46f31`
- License: GPL-3.0 (retained in `LICENSE`)

Local integration changes keep the algorithm intact while making the ROS 2
node fail closed for this workspace: TF publication defaults off, scan and
odometry topics use the SmartCar contract, finite covariances are published,
invalid timestamps/scan-width changes are rejected, and an explicit reset
service is available for physical P-zone resets.
