# RDK .164 Pure-Navigation Field Test (2026-08-04)

## Scope

- Target: `root@172.16.24.164`.
- Route: `nav_only.yaml`, no camera, QR reader, VLM, or speech service.
- Motion path: `velocity_smoother -> direction_guard -> smartcar_safety -> /ackermann_cmd`.
- All testing was supervised. The final software emergency stop is latched.

## Preflight and preparation

- Verified both Nav2 obstacle layers subscribe to `/scan`; scan was about 10 Hz and
  `/odom_combined` about 20 Hz.
- Verified Nav2 lifecycle active, battery 12.6 V, and a zero `/ackermann_cmd` while
  emergency stop was latched.
- Set the EKF pose to P: `odom_combined=(0, 0, yaw=0)` while stopped.
- The RDK copy of `nav_only.yaml` was initially stale with `calibrated: false`.
  It was backed up, then temporarily synchronized with the repository copy solely
  to execute this supervised test. After the test it was restored to
  `calibrated: false` on the RDK.

## Result

The task reported success for all three pure-navigation actions:

| Segment | Result | Duration |
| --- | --- | --- |
| P -> A | succeeded | 12.10 s |
| A -> via_2 -> C1 | succeeded | 26.90 s |
| C1 -> via_1 -> via_3 -> P | succeeded | 32.17 s |

The task node reported `mission_completed`. At the final stop,
`/ackermann_cmd` had zero speed and zero steering. The final fused pose was
approximately `(0.046, 0.065) m` from P with yaw about `0.92 rad`; it is within
the configured return goal envelope.

The return leg emitted one behavior-tree tick overrun and three controller-loop
20 Hz misses. No planner/controller failure or recovery was reported. These
timing warnings remain a follow-up item.

## Blocking observation

The operator observed that the vehicle advanced beyond the physical QR/A point
before Nav2 declared P -> A complete and began the next segment. This is not
explained by a broad A-point arrival envelope:

- `a_task_observe` uses `precise_goal_checker`.
- The active tolerances are `0.12 m` position and `0.15 rad` yaw.
- The RPP `0.8 m` lookahead is a tracking parameter, not the completion test.

Therefore the accepted Nav2 pose and the physical field pose disagree. The
likely causes to validate are P placement/heading, field-coordinate alignment,
wheel/steering scale, and heading accumulation. Do not loosen the goal checker
to conceal this mismatch.

## Next test

Create and run a supervised, single-segment P -> A-only test. The vehicle must
stop at A and be emergency-stopped before any reverse segment can begin. Put the
rear-axle/base-footprint origin on P with the vehicle heading +X, mark the
physical QR/A center, then record the actual displacement and `/odom_combined`
pose when the vehicle stops. Use that measured error to correct coordinates or
odometry before re-enabling a complete route.

## Evidence

The unmodified RDK bringup log was copied to
`/tmp/route_test_bringup_20260804.log` on the development machine. The RDK
route file backup is
`/root/ros2_ws/src/smartcar_nav2/config/waypoints/nav_only.yaml.before-route-test-20260804`.
