# Error Handling

> How failures are surfaced in this workspace.

---

## Overview

The dominant style is ROS runtime error handling, not exception-heavy application error hierarchies.

Common patterns:

- C++ runtime code uses guard clauses and returns `false` when a step cannot proceed.
- ROS logging macros surface the reason near the failing branch.
- Python nodes usually keep running, log what is missing, and wait for upstream data.
- A few legacy planner paths call `exit(1)` for unrecoverable startup or file-load failures.

## Error Types

There are no custom repository-wide exception classes or shared error envelopes.

Instead, the codebase uses:

- `bool` return values for compute or data-read failures
- ROS log macros for operator visibility
- startup termination for a small number of hard failures

Examples:

- `src/FAST_LIO_SUPER/src/slam/map/ikd_tree_map.cpp` returns `false` when plane fitting does not have enough valid points
- `src/FAST_LIO/src/laserMapping.cpp` and `src/FAST_LIO_SUPER/src/laserMapping.cpp` log loop-back or empty-cloud conditions and skip work
- `src/far_planner/src/utility.cpp` catches TF exceptions and logs them with `ROS_ERROR`

## Handling Patterns

### Recoverable runtime conditions

Use a warning or throttled warning and keep the node alive.

Examples:

- `src/greenhouse_nav/scripts/global_to_local_adapter.py` uses `rospy.logwarn_throttle(...)` while waiting for `/state_estimation` or a non-empty path
- `src/far_planner/src/far_planner.cpp` warns when odometry or graph state is not ready
- `src/FAST_LIO/src/laserMapping.cpp` warns and skips scans when no valid points are available

### Expected "not enough data yet" branches

Prefer early return over deep nesting.

Examples:

- `src/FAST_LIO_SUPER/src/slam/map/oct_vox_map.cpp`
- `src/FAST_LIO_LOCALIZATION/src/preprocess.cpp`
- `src/graph_decoder/include/graph_decoder/decoder_node.h`

### Hard startup or file-input failures

Some legacy packages terminate the process directly after logging the failure.

Examples:

- `src/local_planner/src/localPlanner.cpp`
- `src/boundary_handler/src/graph_extractor.cpp`

This pattern exists and must be preserved carefully when touching those flows, but it should not be copied casually into new code.

## Rules For New Code

- Surface failure with an explicit ROS log before returning when the operator needs to understand why progress stopped.
- Prefer early returns and small failure branches over nested "success path only" blocks.
- Use throttled logging inside loops that can wait for upstream state.
- Only terminate the process for unrecoverable startup invariants, such as a required file or hardware channel that makes the node meaningless without it.
- Do not add silent fallback behavior just to make a node "look healthy".

## Anti-Patterns

- Swallowing exceptions or returning `false` without any operator-visible log when the failure is actionable.
- Adding broad `try/except` or `catch (...)` blocks that hide the real fault.
- Introducing a second fallback path that changes behavior silently instead of surfacing the missing dependency.
- Copying legacy `exit(1)` usage into steady-state runtime loops.

## Common Mistakes

- Treating repeated waiting conditions as errors instead of throttled warnings.
- Logging every loop tick instead of using `*_THROTTLE` or `*_ONCE`.
- Replacing existing "skip this scan and continue" behavior with hard termination in sensor pipelines.
