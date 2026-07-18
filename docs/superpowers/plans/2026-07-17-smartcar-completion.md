# SmartCar Competition Software Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete the ROS2 software stack from safe chassis command handling through Nav2, QR/VLM services, mission orchestration, and one-command competition bringup on the RDK X5.

**Architecture:** Keep hardware drivers and Nav2 as lower layers, insert a fail-closed velocity guard before the chassis, expose QR and scene description through explicit ROS services, and let `smartcar_task` own semantic waypoint sequencing. All hardware-dependent values remain ROS parameters; only the EKF publishes `odom_combined -> base_footprint`.

**Tech Stack:** ROS2 Humble/TogetheROS, Python 3/rclpy, C++17/rclcpp, Nav2, robot_localization, zbar_ros, sensor_msgs, std_srvs, rosidl default generators, stdlib unittest, ament/colcon.

## Global Constraints

- Target hardware is OriginCar + RDK X5 8G running ROS2 Humble.
- ROS environment entry is `/opt/tros/humble/setup.bash`; project workspace is `/root/ros2_ws`.
- LiDAR publishes `/scan`; chassis publishes raw `/odom` and `/imu/data_raw`; EKF publishes `/odom_combined` and the sole `odom_combined -> base_footprint` TF.
- Default chassis command path is `/cmd_vel -> smartcar_safety -> /cmd_vel_safe -> Ackermann adapter -> /ackermann_cmd -> origincar_base`.
- The vehicle must fail closed: stale command, stale required sensor, emergency stop, or configured low voltage produces zero velocity.
- Ackermann motion must not use in-place rotation or Nav2 Spin recovery.
- QR recognition uses installed `zbar_ros` (`image` input, `barcode` `std_msgs/String` output).
- VLM requests have an 8 second upper bound and return the fallback text `检测到人物立牌` on timeout or backend failure.
- No runtime dependency on Internet connectivity.
- New behavior is developed test-first; pure logic is isolated from ROS wrappers so it runs under local stdlib `unittest`.

---

### Task 1: Shared Vision Service Interfaces

**Files:**
- Create: `src/smartcar_interfaces/package.xml`
- Create: `src/smartcar_interfaces/CMakeLists.txt`
- Create: `src/smartcar_interfaces/srv/ReadQr.srv`
- Create: `src/smartcar_interfaces/srv/DescribeScene.srv`

**Interfaces:**
- Produces service type `smartcar_interfaces/srv/ReadQr` with request fields `builtin_interfaces/Time not_before`, `float32 timeout_sec`; response fields `bool success`, `string content`, `string status`.
- Produces service type `smartcar_interfaces/srv/DescribeScene` with request fields `builtin_interfaces/Time not_before`, `float32 timeout_sec`, `string prompt`; response fields `bool success`, `bool fallback_used`, `string description`, `string status`.

- [ ] **Step 1: Add an interface contract test**

Create `tests/test_interface_contracts.py` that reads both `.srv` files and asserts the exact non-comment field sequence above.

- [ ] **Step 2: Run the test and verify RED**

Run: `python -m unittest tests.test_interface_contracts -v`

Expected: FAIL because `src/smartcar_interfaces/srv/ReadQr.srv` does not exist.

- [ ] **Step 3: Add the interface package**

Use `rosidl_generate_interfaces` with dependencies `builtin_interfaces`; export `rosidl_default_runtime` and membership in `rosidl_interface_packages`.

- [ ] **Step 4: Verify GREEN**

Run: `python -m unittest tests.test_interface_contracts -v`

Expected: both service contract tests pass.

Run on RDK: `colcon build --packages-select smartcar_interfaces --symlink-install`

Expected: package builds and generated interfaces are visible through `ros2 interface show`.

---

### Task 2: Ackermann Conversion and Chassis Command Watchdog

**Files:**
- Create: `src/origincar/origincar_base/scripts/ackermann_math.py`
- Create: `tests/test_ackermann_math.py`
- Modify: `src/origincar/origincar_base/scripts/cmd_vel_to_ackermann_drive.py`
- Create: `src/origincar/origincar_base/include/origincar_base/command_watchdog.hpp`
- Create: `src/origincar/origincar_base/test/test_command_watchdog.cpp`
- Modify: `src/origincar/origincar_base/include/origincar_base/origincar_base.h`
- Modify: `src/origincar/origincar_base/src/origincar_base.cpp`
- Modify: `src/origincar/origincar_base/launch/base_serial.launch.py`
- Modify: `src/origincar/origincar_base/launch/origincar_bringup.launch.py`
- Modify: `src/origincar/origincar_base/CMakeLists.txt`
- Modify: `src/origincar/origincar_base/package.xml`

**Interfaces:**
- `steering_angle(linear_velocity, angular_velocity, wheelbase, max_steering_angle) -> float` returns zero when either velocity is effectively zero and otherwise clamps `atan(wheelbase * angular_velocity / linear_velocity)`.
- `CommandWatchdog(timeout_sec)` exposes `mark_command(now_sec)` and `consume_stop(now_sec)`; `consume_stop` returns true once for each active-to-timed-out transition.
- Launch defaults: `akmcar=true`, wheelbase `0.189`, maximum steering angle `0.45` rad, command timeout `0.35` s, chassis input topic `/cmd_vel_safe`.

- [ ] **Step 1: Write failing Python conversion tests**

Cover straight motion, left/right sign, saturation, reverse motion, and zero-linear-velocity behavior.

- [ ] **Step 2: Verify Python RED**

Run: `python -m unittest tests.test_ackermann_math -v`

Expected: FAIL because `ackermann_math.py` does not exist.

- [ ] **Step 3: Implement conversion and parameterize the ROS adapter**

The adapter declares `wheelbase`, `max_steering_angle`, `input_topic`, `output_topic`, and `frame_id`; it uses the pure conversion function and never emits an unclamped steering angle.

- [ ] **Step 4: Verify Python GREEN**

Run: `python -m unittest tests.test_ackermann_math -v`

Expected: all conversion tests pass.

- [ ] **Step 5: Write failing C++ watchdog tests**

Cover no-command behavior, pre-timeout behavior, one-shot timeout, no repeated stop, and re-arming after a new command.

- [ ] **Step 6: Verify C++ RED on RDK**

Run: `colcon build --packages-select origincar_base --cmake-args -DBUILD_TESTING=ON`

Expected: FAIL because `command_watchdog.hpp` does not exist.

- [ ] **Step 7: Implement the watchdog in the driver**

Both Twist and Ackermann callbacks call `mark_command`. The control loop processes callbacks before sensor reads, sends one zero command after timeout, uses a configurable serial read timeout no larger than 100 ms, and updates odometry integration time only after a valid sensor frame.

- [ ] **Step 8: Verify C++ GREEN**

Run: `colcon build --packages-select origincar_base --symlink-install --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select origincar_base && colcon test-result --verbose`

Expected: watchdog tests pass with zero failures.

---

### Task 3: Fail-Closed Velocity Safety Package

**Files:**
- Create: `src/smartcar_safety/package.xml`
- Create: `src/smartcar_safety/setup.py`
- Create: `src/smartcar_safety/setup.cfg`
- Create: `src/smartcar_safety/resource/smartcar_safety`
- Create: `src/smartcar_safety/smartcar_safety/__init__.py`
- Create: `src/smartcar_safety/smartcar_safety/guard.py`
- Create: `src/smartcar_safety/smartcar_safety/safety_node.py`
- Create: `src/smartcar_safety/config/safety.yaml`
- Create: `src/smartcar_safety/launch/smartcar_safety.launch.py`
- Create: `src/smartcar_safety/test/test_guard.py`
- Modify: `src/smartcar_bringup/package.xml`
- Modify: `src/smartcar_bringup/launch/smartcar_bringup.launch.py`

**Interfaces:**
- Subscribe `/cmd_vel`, `/scan`, `/odom_combined`, `/PowerVoltage`.
- Publish `/cmd_vel_safe` and `/smartcar/safety/status`.
- Provide `/smartcar/safety/emergency_stop` as `std_srvs/SetBool`.
- Pure `SafetyGuard.evaluate(now_sec)` returns `{allowed, reason}` from configured command/sensor ages, voltage, and emergency-stop state.
- Defaults: command timeout `0.30` s, scan timeout `0.35` s, odom timeout `0.35` s, minimum voltage disabled with `0.0`, publish frequency `20.0` Hz, `require_scan=true`, `require_odom=true`.

- [ ] **Step 1: Write failing guard tests**

Cover startup fail-closed, healthy pass-through, stale command, stale scan, stale odom, emergency stop, optional sensor disable, and low voltage.

- [ ] **Step 2: Verify RED**

Run: `python -m unittest discover -s src/smartcar_safety/test -v`

Expected: FAIL because `smartcar_safety.guard` does not exist.

- [ ] **Step 3: Implement pure guard logic and ROS wrapper**

The wrapper copies a permitted Twist or emits an all-zero Twist. Emergency stop is latched until explicitly cleared. Status changes are published immediately and repeated at 1 Hz while blocked.

- [ ] **Step 4: Verify GREEN**

Run: `python -m unittest discover -s src/smartcar_safety/test -v`

Expected: all guard tests pass.

Run on RDK: `colcon build --packages-select smartcar_safety smartcar_bringup --symlink-install`

Expected: both packages build.

---

### Task 4: Humble/Ackermann Nav2 Hardening

**Files:**
- Modify: `src/smartcar_nav2/config/nav2_params.yaml`
- Modify: `src/smartcar_nav2/config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
- Modify: `src/smartcar_nav2/config/waypoints/default_waypoints.yaml`
- Modify: `src/smartcar_nav2/test/test_smartcar_nav2_launch.py`
- Create: `tests/test_nav2_contracts.py`
- Modify: `src/smartcar_nav2/package.xml`

**Interfaces:**
- Use `behavior_server` with `nav2_behaviors/BackUp`, `nav2_behaviors/DriveOnHeading`, and `nav2_behaviors/Wait`; no Spin plugin or BT action.
- RPP has `use_rotate_to_heading=false` and `allow_reversing=true`.
- Planner is `nav2_smac_planner/SmacPlannerHybrid`, `motion_model_for_search=DUBIN`, `minimum_turning_radius=0.40`.
- Both costmaps use polygon footprint `[[0.168,0.112],[0.168,-0.112],[-0.168,-0.112],[-0.168,0.112]]` and no `robot_radius`.
- Waypoint pause duration is 100 ms because mission tasks own all semantic waits.
- `stop_on_failure=true`; adjacent competition waypoints remain within the rolling global costmap.

- [ ] **Step 1: Write failing static contract tests**

Parse YAML/XML and assert the interfaces above, valid waypoint quaternions, and absence of `<Spin`.

- [ ] **Step 2: Verify RED**

Run: `python -m unittest tests.test_nav2_contracts -v`

Expected: FAIL on the existing `recoveries_server`, `Spin`, and `use_rotate_to_heading=true` values.

- [ ] **Step 3: Update Nav2 configuration and launch test**

The launch test must provide temporary static `odom_combined -> base_footprint`, a zero odometry publisher, and an empty LaserScan publisher, then assert all managed Nav2 lifecycle nodes reach `active` rather than matching a log string.

- [ ] **Step 4: Verify GREEN locally and on RDK**

Run locally: `python -m unittest tests.test_nav2_contracts -v`

Run on RDK: `colcon build --packages-select smartcar_nav2 --symlink-install && colcon test --packages-select smartcar_nav2 && colcon test-result --verbose`

Expected: contract and launch tests pass.

---

### Task 4.5: Localization, EKF, and Odometry Hardening

**Files:**
- Create: `src/origincar/origincar_base/include/origincar_base/sensor_calibration.hpp`
- Create: `src/origincar/origincar_base/include/origincar_base/serial_frame.hpp`
- Create: `src/origincar/origincar_base/test/test_sensor_calibration.cpp`
- Create: `src/origincar/origincar_base/test/test_serial_frame.cpp`
- Modify: `src/origincar/origincar_base/include/origincar_base/origincar_base.h`
- Modify: `src/origincar/origincar_base/src/origincar_base.cpp`
- Modify: `src/origincar/origincar_base/launch/base_serial.launch.py`
- Modify: `src/origincar/origincar_base/config/ekf.yaml`
- Modify: `src/origincar/origincar_base/CMakeLists.txt`
- Modify: `src/smartcar_safety/smartcar_safety/guard.py`
- Modify: `src/smartcar_safety/smartcar_safety/safety_node.py`
- Modify: `src/smartcar_safety/config/safety.yaml`
- Modify: `src/smartcar_safety/test/test_guard.py`
- Create: `tests/test_localization_contracts.py`

**Interfaces:**
- EKF uses `two_d_mode=true`, `sensor_timeout=0.25`, and remains the sole publisher of `odom_combined -> base_footprint`.
- Wheel odometry fuses only calibrated `vx` plus the Ackermann nonholonomic `vy=0` constraint. It does not fuse wheel-integrated pose or wheel-derived yaw rate; `odom0_differential=false`.
- Raw IMU marks orientation unavailable with `orientation_covariance[0]=-1`; EKF fuses only calibrated `angular_velocity.z`, not the gyro-integrated yaw.
- `/odom` pose/twist and `/imu/data_raw` covariance values are finite, nonzero, configurable, and conservative until measured calibration data replaces the defaults. Initial EKF covariance must not claim near-perfect certainty.
- Chassis parameters expose finite calibration values for longitudinal/lateral/yaw velocity scale, gyro-z scale and bias, steering command scale and offset. The same calibrated velocity values are published and integrated.
- Chassis serial input is parsed as a persistent byte stream; partial reads, arbitrary read offsets, payload marker bytes, and corrupted frames cannot permanently desynchronize frame recovery.
- The safety gate independently monitors finite raw `/odom` and fused `/odom_combined`; a continuing EKF prediction cannot hide a stale chassis sensor stream. A raw-odometry timeout latches a localization fault until the reset workflow verifies `/set_pose` through a newer finite filtered-odometry sample and explicitly clears the latch.
- Competition initialization requires the vehicle at the P-zone origin with its nose aligned to `+X`. The later task reset adapter uses robot_localization `/set_pose` to restore `(x, y, yaw)=(0, 0, 0)` only after navigation has stopped.
- Camera optical flow is not a release dependency. If later enabled, it is a separately timestamped `TwistWithCovarianceStamped` source and fuses velocity only; LiDAR remains excluded from localization.

- [ ] **Step 1: Write failing localization and safety contracts**

Parse `ekf.yaml` and assert the exact fusion vectors, timeout, frames, TF ownership, covariance bounds, and absence of correlated pose/yaw inputs. Add guard tests for missing and stale raw odometry. Add C++ tests for calibration, finite-parameter validation, covariance construction, and consistent integration inputs.

- [ ] **Step 2: Verify RED**

Run locally: `python -m unittest tests.test_localization_contracts -v`

Run on RDK: `colcon test --packages-select origincar_base smartcar_safety`

Expected: failures on the existing zero odometry covariance, raw IMU orientation, duplicated EKF inputs, two-second timeout, hard-coded scale factors, and missing raw-odometry safety heartbeat.

- [ ] **Step 3: Implement calibrated sensor publication and EKF fusion**

Apply calibration once when decoding a valid serial frame, fill ROS covariance arrays from validated parameters, publish raw IMU orientation as unavailable, and reduce EKF inputs to independent velocity measurements. Extend the safety guard and launch configuration with `require_raw_odom=true` and a default timeout no greater than `0.25 s`.

- [ ] **Step 4: Verify GREEN and reset behavior**

Run local pure/contract tests, then build and test `origincar_base`, `smartcar_safety`, and `smartcar_bringup` on the RDK. In an isolated ROS domain, launch only the EKF with synthetic zero `/odom` and `/imu/data_raw`, verify finite `/odom_combined`, call `/set_pose`, and confirm the filtered pose resets without starting the chassis node or publishing a velocity command.

Expected: zero failed tests; all synthetic outputs are finite; stopping raw `/odom` leaves the safety guard blocked even if filtered odometry continues.

---

### Task 5: QR and VLM Vision Services

**Files:**
- Create: `src/smartcar_vision/package.xml`
- Create: `src/smartcar_vision/setup.py`
- Create: `src/smartcar_vision/setup.cfg`
- Create: `src/smartcar_vision/resource/smartcar_vision`
- Create: `src/smartcar_vision/smartcar_vision/__init__.py`
- Create: `src/smartcar_vision/smartcar_vision/timed_sample.py`
- Create: `src/smartcar_vision/smartcar_vision/vlm_backend.py`
- Create: `src/smartcar_vision/smartcar_vision/vision_node.py`
- Create: `src/smartcar_vision/config/vision.yaml`
- Create: `src/smartcar_vision/launch/smartcar_vision.launch.py`
- Create: `src/smartcar_vision/test/test_timed_sample.py`
- Create: `src/smartcar_vision/test/test_vlm_backend.py`
- Create: `src/smartcar_vision/test/test_vision_service.py`
- Create: `tests/test_vision_launch_contracts.py`

**Interfaces:**
- Launch selector `camera_driver:=aurora|usb|mipi|none` defaults to the installed Aurora930 driver. Aurora enables only 15 FPS RGB and publishes `/aurora/rgb/image_raw`; optional USB and MIPI modes remain explicit deployment fallbacks.
- Run `zbar_ros/barcode_reader` with `throttle_repeated_barcodes=0.0`, remap `image` from the configured image topic and `barcode` to `/barcode`. The default image topic is `/aurora/rgb/image_raw`.
- Subscribe `/barcode` (`std_msgs/String`) and the configurable image topic (`sensor_msgs/Image`, sensor-data QoS).
- Provide `/smartcar/vision/read_qr` and `/smartcar/vision/describe_scene` using Task 1 services.
- `TimedSampleBuffer.wait_for(not_before_ns, timeout_sec)` uses callback receipt time in integer ROS nanoseconds, a `threading.Condition`, and a monotonic wait deadline; it returns only samples received at or after the requested time.
- VLM backend modes are `command`, `static`, and `disabled`. `command` accepts an argv list, expands `{image}` and `{prompt}` per argument without invoking a shell, captures stdout, enforces the remaining request deadline, and terminates the process group on timeout. `static` returns configured text for bench tests. `disabled` returns a backend-unavailable error.
- Any VLM timeout or error returns `success=true`, `fallback_used=true`, description `检测到人物立牌`, and a diagnostic status; absence of a fresh image returns `success=false`.
- The whole DescribeScene operation has one finite deadline capped at 8 seconds, including fresh-image wait, JPEG encoding, and backend work. Temporary JPEG files use mode `0600` and are removed on every exit path.

- [x] **Step 1: Write failing sample-buffer and backend tests**

Cover fresh/stale/equal-time sample selection, concurrent wake-up, command argument expansion without shell interpretation, process-group timeout, static/disabled backends, one-deadline fallback selection, and temporary-file cleanup. Add launch contracts for the Aurora default, disabled depth streams, selected image topic, and zbar remaps.

- [x] **Step 2: Verify RED**

Run: `python -m unittest discover -s src/smartcar_vision/test -v`

Expected: FAIL because vision modules do not exist.

- [x] **Step 3: Implement pure logic, ROS services, and launch**

Use a `MultiThreadedExecutor` with at least three threads, reentrant subscription callbacks, and a mutually-exclusive service callback group so service waits do not block samples and two VLM requests cannot run concurrently. Convert the selected image to JPEG through `cv_bridge`; store it under a configurable runtime directory and remove it after backend completion.

- [x] **Step 4: Verify GREEN and RDK build**

Run locally: `python -m unittest discover -s src/smartcar_vision/test -v`

Run on RDK: `colcon build --packages-select smartcar_interfaces smartcar_vision --symlink-install`

Expected: tests and build pass; `ros2 service type` reports both custom services.

---

### Task 6: Mission State Machine and Waypoint Execution

**Files:**
- Create: `src/smartcar_task/package.xml`
- Create: `src/smartcar_task/setup.py`
- Create: `src/smartcar_task/setup.cfg`
- Create: `src/smartcar_task/resource/smartcar_task`
- Create: `src/smartcar_task/smartcar_task/__init__.py`
- Create: `src/smartcar_task/smartcar_task/waypoints.py`
- Create: `src/smartcar_task/smartcar_task/mission.py`
- Create: `src/smartcar_task/smartcar_task/task_node.py`
- Create: `src/smartcar_task/config/task.yaml`
- Create: `src/smartcar_task/launch/smartcar_task.launch.py`
- Create: `src/smartcar_task/test/test_waypoints.py`
- Create: `src/smartcar_task/test/test_mission.py`

**Interfaces:**
- Load semantic waypoints from the existing Nav2 YAML schema.
- Execute `/follow_waypoints` with one pose per goal, waiting for action success before running that waypoint task.
- Provide `/smartcar/task/start`, `/smartcar/task/stop`, and `/smartcar/task/reset` (`std_srvs/Trigger`).
- `/smartcar/task/reset` is accepted only after navigation is stopped; it calls robot_localization `/set_pose` with the P-zone origin and zero yaw, verifies a newer finite `/odom_combined`, then calls `/smartcar/safety/clear_localization_fault` before returning to `IDLE`.
- Publish `/smartcar/task/state`, `/smartcar/output/text`, and `/smartcar/output/speech` (`std_msgs/String`).
- Mission states are `IDLE`, `WAITING_FOR_SERVERS`, `NAVIGATING`, `RUNNING_QR`, `RUNNING_VLM`, `COMPLETED`, `STOPPED`, `FAILED`.
- Task policies: `start`, `corridor`, `loop`, and `return` require no vision call; `qr` calls ReadQr after a configurable 2 second settle delay and retries once; `vlm` calls DescribeScene with an 8 second timeout and publishes returned/fallback text.

- [ ] **Step 1: Write failing waypoint/parser and state-machine tests**

Cover malformed YAML, unknown task, non-normalized quaternion, success path, navigation retry, QR retry, VLM fallback, stop request, and reset after terminal state. Use fake ports, not ROS mocks.

- [ ] **Step 2: Verify RED**

Run: `python -m unittest discover -s src/smartcar_task/test -v`

Expected: FAIL because task modules do not exist.

- [ ] **Step 3: Implement ports-and-adapters mission logic and ROS node**

Pure mission logic depends on interfaces `Navigator.navigate(waypoint)`, `Vision.read_qr(not_before, timeout)`, `Vision.describe_scene(not_before, timeout, prompt)`, `Localization.reset_origin()`, `Clock.now/sleep`, and `Output.publish`. The ROS localization adapter owns the ordered `/set_pose` -> newer finite `/odom_combined` -> safety localization-fault clear sequence. The ROS node adapts Nav2 actions/services to those interfaces and runs the mission on a worker thread so service callbacks remain responsive.

- [ ] **Step 4: Verify GREEN and RDK build**

Run locally: `python -m unittest discover -s src/smartcar_task/test -v`

Run on RDK: `colcon build --packages-select smartcar_interfaces smartcar_task --symlink-install`

Expected: tests and package build pass.

---

### Task 7: Competition Bringup and Configuration Integration

**Files:**
- Create: `src/smartcar_bringup/launch/smartcar_system.launch.py`
- Modify: `src/smartcar_bringup/launch/smartcar_bringup.launch.py`
- Modify: `src/smartcar_bringup/config/bringup_coord.yaml`
- Modify: `src/smartcar_bringup/CMakeLists.txt`
- Modify: `src/smartcar_bringup/package.xml`
- Create: `tests/test_system_contracts.py`
- Modify: `README.md`
- Modify: `docs/deployment/rdk-environment-setup.md`

**Interfaces:**
- `smartcar_system.launch.py` composes base/LiDAR/safety, Nav2, camera/vision, and task packages.
- Launch switches: `use_lidar`, `use_obstacle`, `use_safety`, `use_nav`, `use_camera`, `use_vision`, `use_task`, `autostart_mission`, and `use_sim_time`.
- Default TF is `base_footprint -> base_link` identity; laser and camera offsets are explicit launch/config parameters rather than hidden constants.
- The system does not autostart vehicle motion by default; a mission begins through `/smartcar/task/start` unless `autostart_mission=true` is explicitly supplied.

- [ ] **Step 1: Write failing system contract tests**

Assert package dependencies, launch switches, safe command topic wiring, identity base transform, and non-autostart default.

- [ ] **Step 2: Verify RED**

Run: `python -m unittest tests.test_system_contracts -v`

Expected: FAIL because `smartcar_system.launch.py` does not exist.

- [ ] **Step 3: Implement launch composition and documentation**

Document exact build, bench launch, full launch, start/stop/reset, emergency-stop, and bag-recording commands. Mark waypoint coordinates and measured sensor extrinsics as deployment calibration data, not code placeholders.

- [ ] **Step 4: Verify GREEN**

Run: `python -m unittest tests.test_system_contracts -v`

Expected: all system contract tests pass.

---

### Task 8: Full Deployment and Non-Motion Verification

**Files:**
- Modify only files required by failures found during verification.

**Interfaces:**
- Local and RDK workspaces contain identical source trees through `scripts/sync_to_rdk.py push`.
- All packages build together in `/root/ros2_ws`.
- Bench verification never publishes a nonzero velocity.

- [ ] **Step 1: Run the complete local suite**

Run: `python -m unittest discover -s tests -v`

Run each package pure test directory with `python -m unittest discover`.

Expected: zero failures.

- [ ] **Step 2: Sync and build the complete RDK workspace**

Run locally: `python scripts/sync_to_rdk.py push`

Run on RDK: `source ~/source_env.sh && cd /root/ros2_ws && colcon build --symlink-install`

Expected: all packages build.

- [ ] **Step 3: Run ROS tests**

Run on RDK: `colcon test && colcon test-result --verbose`

Expected: zero failed tests.

- [ ] **Step 4: Run a no-motion system smoke test**

Launch with camera disabled and emergency stop asserted before Nav2 activation. Verify `/scan`, `/odom_combined`, `/cmd_vel_safe`, safety status, Nav2 lifecycle states, vision/task services, and that `/cmd_vel_safe` remains exactly zero.

- [ ] **Step 5: Record the physical-test gate**

The software milestone is ready for wheel-off-ground testing only after all automated checks pass. Ground motion remains gated on measured TF/extrinsics, steering calibration, a human-accessible emergency stop, and explicit operator approval.
