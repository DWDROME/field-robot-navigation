# Quality Guidelines

> Practical quality rules for changing this workspace.

---

## Overview

This repository mixes upstream-derived robotics packages, local integration code, and research-heavy documentation. Quality work here means "preserve runtime contracts and verify the touched package", not "apply generic web-app tooling".

## Required Patterns

- Keep changes inside the owning ROS package whenever possible.
- Preserve public ROS contracts unless the task explicitly changes them:
  - package names
  - node names
  - launch entry names
  - topic names
  - frame names
- Prefer explicit launch wiring and parameter loading over hidden defaults.
- Keep file-based docs and config close to the code they describe.
- Follow the language and build standard already used by the package:
  - `src/FAST_LIO_SUPER/CMakeLists.txt` uses C++17
  - several legacy packages still use older catkin/CMake baselines
  - `src/greenhouse_nav/CMakeLists.txt` installs Python nodes through `catkin_install_python`

## Forbidden Patterns

- Silent fallback logic that hides missing topics, frames, files, or hardware state.
- Flattening package boundaries by dropping unrelated helper code into another package "because it builds there".
- Treating alternate/reference stacks such as `MoveBase3D` as if they were the current mainline without checking the docs first.
- Repo-wide refactors that rewrite upstream-derived packages without a task that justifies the blast radius.
- Inventing frontend, database, or service-layer rules that do not exist in this workspace.

## Validation Requirements

There is little to no automated test coverage in the current workspace. Use the smallest real verification that matches the change.

Typical order:

1. documentation link and explicit evidence-path checks
2. structured parse checks for changed YAML, JSON, XML/launch, and RViz files
3. target launch assembly checks such as:
   - `roslaunch greenhouse_nav greenhouse_navigation.launch --nodes`
   - `roslaunch greenhouse_nav greenhouse_mapping.launch --nodes`
4. `catkin_make` after sourcing ROS and the workspace when code, build
   metadata, messages, or installed resources changed
5. package-specific smoke checks when the task touches one node or one bridge
   script
6. live graph checks, when a ROS graph is available:
   - `rosnode list` / `rosnode info`
   - `rostopic type` / `rostopic info` / targeted `rostopic echo`
   - TF parent/child and broadcaster-owner inspection

Common command baseline:

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Evidence for current expectations:

- `README.md` documents `catkin_make` and launch-driven workflows
- `CLAUDE.md` states that validation is mostly Docker or manual `roslaunch` smoke work
- `src/FAST_LIO/package.xml` declares `rostest`, but the repository does not currently present a maintained automated test suite across packages

## Validation Matrix

| Change | Minimum evidence |
|--------|------------------|
| Trellis/doc inventory only | Markdown links, explicit repository paths, package counts, task validation |
| YAML/JSON/XML/RViz | Structured parse plus enabled resource existence |
| Launch wiring | Recursive `roslaunch ... --nodes`; inspect expanded resources |
| Message/build dependency | `catkin_make`, generated header, direct consumers |
| Topic/remap/type | Source producer/consumer plus live `rostopic` when available |
| TF semantics | Broadcaster source plus live TF owner/parent-child check when available |
| Runtime failure behavior | Exercise missing topic/file/device path and inspect log/exit/continue/stop behavior |
| Hardware workflow | Target-host/hardware test; bag or no-hardware smoke is not equivalent |

When hardware is unavailable, use `system_greenhouse_bag.launch` or a targeted
node smoke where possible and explicitly state the unverified hardware layer.

## License And Reuse Boundaries

Not every similar package is safe to merge or reuse freely.

- `src/FAST_LIO_SUPER/` is the active SLAM research/mainline extension area for the pluggable backend work
- `src/slam_octvox/` is vendored support for that line
- `src/FAST_LIO_LOCALIZATION/` has conflicting BSD manifest, GPLv2 directory
  license, and prose evidence; treat it as an unresolved separate boundary

Rule:

- inspect license and package role before reusing code across these boundaries
- do not assume "same workspace" means "safe to merge"
- do not trust `package.xml` license tags alone for the FAST-LIO family; check
  provenance, `LICENSE`, notices, package-local docs, and file headers when
  reuse matters

## Review Checklist

- Did the change land in the correct ROS package?
- Did it preserve launch, topic, TF, and config contracts?
- Did it keep one source of truth for config keys and parameter loading?
- Did it avoid silent fallback behavior?
- Did it preserve or improve operator-visible logs around the changed behavior?
- Did it update docs when the task changed package purpose, launch entry semantics, or operator workflow?
- Did it validate with the smallest relevant real command?
- Did every stated validation claim name the exact layer actually exercised?
- Did documentation-only work avoid claiming a ROS build or hardware test?
- Did enabled resource paths exist after launch argument expansion?

## Common Mistakes

- Changing a launch argument or topic name in one file but not in the integration package that wires it together.
- Editing `FAST_LIO`, `FAST_LIO_SUPER`, and `FAST_LIO_LOCALIZATION` as if they were interchangeable. They are related, but they are not the same package.
- Forgetting that `MoveBase3D` contains several child packages rather than one module.
- Assuming missing automated tests means "no validation needed". In this repo it means build and launch checks matter more.
