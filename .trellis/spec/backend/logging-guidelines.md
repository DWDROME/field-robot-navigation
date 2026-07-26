# Logging Guidelines

> Runtime logging patterns used by the ROS nodes in this workspace.

---

## Overview

Logging is ROS-native:

- C++ packages use `ROS_INFO`, `ROS_WARN`, `ROS_ERROR`, plus throttled/once variants
- Python packages use `rospy.loginfo`, `rospy.logwarn`, and throttled warning helpers

There is no structured JSON logging layer.

## Log Levels

### `INFO`

Use for startup confirmation, major mode selection, or one-time state transitions.

Examples:

- `src/greenhouse_nav/scripts/cmd_vel_stamped_bridge.py` logs node startup
- `src/greenhouse_nav/scripts/localization_manager.py` logs the computed `body -> base_link` mount and node startup
- `src/FAST_LIO_SUPER/src/laserMapping.cpp` logs selected `slam_core/map_backend`

### `WARN`

Use when the node can continue but the data is incomplete, delayed, or suspicious.

Examples:

- `src/FAST_LIO/src/laserMapping.cpp` warns on "No Effective Points" and skipped scans
- `src/far_planner/src/far_planner.cpp` warns while waiting for odometry or graph readiness
- `src/greenhouse_nav/scripts/global_to_local_adapter.py` warns while upstream path/state is still missing

### `ERROR`

Use when an operation failed and the operator likely needs to intervene or inspect upstream state.

Examples:

- `src/far_planner/src/utility.cpp` logs TF lookup exceptions with `ROS_ERROR`
- `src/FAST_LIO_SUPER/src/laserMapping.cpp` logs lidar loop-back buffer faults
- `src/dlrobot_robot/src/dlrobot_robot.cpp` logs serial port failures

## Repeated Logging

In loops or callbacks that may wait for a long time, use:

- `ROS_WARN_THROTTLE`
- `ROS_ERROR_THROTTLE`
- `ROS_WARN_ONCE`
- `rospy.logwarn_throttle`

This is already the pattern in planner and adapter code. Keep it that way.

## What To Log

Log facts that help the next operator or AI reason about runtime state:

- which mode or backend was selected
- which topic or frame is missing or mismatched
- why a scan or plan step was skipped
- hardware connection failures
- TF lookup failures
- one-time mount or transform parameters when they are operationally important

## What Not To Log By Default

- high-volume per-point or per-message debug noise in hot paths
- duplicate loop messages without throttling
- silent "success" spam every cycle

The repo contains a few legacy exceptions, such as repeated hex dumps in `src/dlrobot_robot/src/dlrobot_robot.cpp`. Treat those as hardware-debug-specialized behavior, not the default logging style for new work.

## Format Preferences

- Keep messages operational and concrete.
- Mention the subsystem prefix when the package already does so, for example `GP:` or `FARMaster:`.
- Include the failing dependency in the message when possible: topic, frame, map file, or serial port.

## Common Mistakes

- Downgrading actionable failures to debug logs that nobody will see in normal ROS runs.
- Logging the same missing-input warning every cycle instead of throttling it.
- Removing existing startup logs that operators use to confirm node wiring.
