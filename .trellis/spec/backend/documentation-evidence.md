# Documentation Evidence Rules

> How to resolve disagreement between runtime code, launch assembly, package
> metadata, and retained project prose.

---

## Evidence Priority

For claims about the current greenhouse runtime, use this order:

```text
recursively assembled user-entry launch
> actual source interfaces plus package.xml/CMakeLists.txt
> configuration files
> current command output, logs, and validation results
> README and CLAUDE prose
> historical research notes or future design drafts
```

This priority is scoped to runtime and build facts. For licensing, apply the
separate evidence process in
[Licensing And Provenance Evidence](./licensing-and-provenance.md); no single
manifest or prose file is sufficient.

## Status Labels

Use one of these labels when summarizing retained documentation:

| Label | Meaning |
|-------|---------|
| `current` | Matches the active user-entry launch and implementation |
| `standalone` | Valid for a package-local entry but not the greenhouse mainline |
| `legacy` | Retained behavior or entry superseded by the current assembly |
| `future/research` | Proposed architecture, experiment, or roadmap |
| `unverified` | Plausible claim without current local command/runtime evidence |

Do not flatten different engineering directions into one "conflict".
`FAST_LIO_SUPER` research documentation can be valid for the SUPER branch
while still not describing today's greenhouse launch.

## Scenario: Use Documentation As Engineering Evidence

### 1. Scope / Trigger

- Writing specs, changing an entrypoint, diagnosing a mismatch, or reporting
  build/runtime validation.

### 2. Signatures

- Current launch evidence starts at
  `src/greenhouse_nav/launch/greenhouse_navigation.launch` or
  `greenhouse_mapping.launch`.
- Package identity comes from `package.xml`.
- Executable and generated-message behavior comes from both
  `CMakeLists.txt` and source includes/targets.

### 3. Contracts

- Trace includes and remaps before quoting a package README as system
  behavior.
- Distinguish "built", "launch enumerated", "no-hardware smoke tested",
  "bag replayed", and "validated on hardware".
- Preserve historical docs unless the task explicitly owns rewriting them;
  record their status and conflict in project specs instead.
- A path or command in documentation must exist or be clearly marked as an
  example/pattern.

### 4. Validation & Error Matrix

| Claim | Minimum evidence |
|-------|------------------|
| "Current navigation starts node X" | Recursive launch assembly |
| "Topic X has type Y" | Publisher/subscriber source or live `rostopic type` |
| "Frame A -> B is broadcast" | TF broadcaster source or live TF inspection |
| "Package X depends on Y" | Manifest/CMake/source usage, classified as build or runtime |
| "Build passes" | Successful build output from this checkout |
| "Hardware workflow works" | Target-hardware evidence; smoke/bag evidence is not enough |
| "License is X" | Provenance and license evidence review, including conflicts |

### 5. Good / Base / Bad Cases

- Good: label a SUPER backend paper `future/research` and separately document
  the current greenhouse baseline launch.
- Base: retain README/CLAUDE files as useful orientation with lower priority
  than executable evidence.
- Bad: copy a stale topic type or include graph into a new spec because it
  appears in a package-local CLAUDE file.

### 6. Tests Required

- Check Markdown relative links.
- Check every explicit repository evidence path that is presented as an
  existing file.
- Re-run package counts and manifest/license inventories when those numbers
  are stated.
- Run the command that supports a validation claim, or mark the claim
  unverified.

### 7. Wrong vs Correct

#### Wrong

- "README says the stack is validated, therefore the real robot loop is
  verified."
- "A future LiLoc/backend document proves the current greenhouse runtime uses
  that design."

#### Correct

- State the exact completed layer: compile, launch enumeration, no-hardware
  smoke, bag replay, or hardware closed loop.
- Label future documents and use current launch/source files for present
  contracts.

## Known Documentation Drift

- `src/greenhouse_nav/CLAUDE.md` retains an obsolete flat navigation assembly
  and gives incorrect types for `/way_point` and `/navigation_boundary`.
  Current types are `geometry_msgs/PointStamped` and
  `geometry_msgs/PolygonStamped`.
- Top-level `CLAUDE.md` contains a build command under
  `/mnt/d/test_move3d/test_move3d`; this checkout's workspace root is
  `/mnt/d/test_move3d`.
- `src/FAST_LIO_SUPER/docs/ARCHITECTURE.md` says the executable is not linked
  to `slam_core`, while the current CMake target is linked.
- `src/FAST_LIO_SUPER/docs/SLAM_CORE_API.md` marks the OctVox factory as
  future work although the implementation and target are present.
- SUPER backend and LiLoc reports describe research direction, not a completed
  greenhouse runtime migration.
- Existing build/no-hardware smoke claims must not be promoted to real-robot
  closed-loop or same-bag numerical-regression claims.

## Primary Evidence

- `src/greenhouse_nav/launch/`
- `src/greenhouse_nav/scripts/`
- package manifests and CMake files under `src/`
- `README.md`
- `CLAUDE.md`
- package-local `CLAUDE.md` files
- `src/FAST_LIO_SUPER/docs/`
