# Database Guidelines

> File-based persistence and asset classification for this workspace.

---

## Overview

This repository does not have an application database.

There is no:

- SQL database
- ORM
- migration system
- repository layer
- API persistence model

Persistent state is file-based and ROS-native instead. Use
[Configuration And Asset Contracts](./configuration-and-assets.md) for
configuration/static resources and
[Runtime Artifact Contracts](./runtime-artifacts.md) for generated outputs.

## What Counts As Persistence Here

The workspace contains four different persistence classes:

- source configuration: YAML, JSON, launch, and RViz profiles;
- static algorithm input: local-planner PLY/TXT and selected FAR data;
- runtime-generated output: PCD scans, logs, timing CSV, graph saves, bags;
- deployment assets: promoted maps, robot descriptions, device/udev settings.

Runtime state flows through:

- ROS parameters
- ROS topics
- TF
- launch-time arguments

## Query Patterns

There are no database query patterns to follow.

Instead:

- read configuration from ROS params or YAML already loaded by launch files;
- pass runtime data through topics;
- preserve versioned algorithm inputs;
- write generated output to an explicit runtime location where practical;
- promote maps or graphs into deployment assets only after provenance and
  consumer validation.

Examples:

- `src/greenhouse_nav/launch/system_greenhouse_base.launch` loads `$(find fast_lio)/config/mid360.yaml` through `<rosparam command="load" ... />`
- `src/greenhouse_nav/scripts/localization_manager.py` reads runtime behavior from ROS params such as `~publish_rate_hz` and topic names

## Migrations

There is no migration process today.

If a task changes data shape, the common patterns are:

- update YAML keys in the owning package,
- update launch files that load those keys,
- update code that reads the params,
- and document any operator-visible change in package docs or top-level docs.

## Naming Conventions

- Configuration files use descriptive, mode-specific names such as `mid360.yaml`, `avia.yaml`, and `default.yaml`.
- Saved map files use the upstream package-local naming that the runtime
  currently expects, for example `PCD/scans.pcd`; this does not make the file
  tracked or guaranteed on another host.
- Message package names and message file names are part of the ROS contract. Treat them more like schema names than casual filenames.

## Rules

- Do not invent a database abstraction for a robotics workspace that already uses ROS params, YAML, and files.
- Keep source configuration and static inputs near the package that owns
  them. Prefer a separate explicit runtime output path for newly designed
  generated artifacts.
- If you need a new config key, prefer extending existing YAML and launch wiring instead of adding a second source of truth.
- If you need structured shared runtime data, prefer a ROS message or service definition over a pseudo-database file.

## Common Mistakes

- Assuming "state" means a database table. In this repo it usually means topic state, TF state, or YAML-loaded params.
- Adding a second config path when a package already has a canonical `config/*.yaml`.
- Treating every PCD/PLY/TXT/VGH file as generated. Some are required,
  versioned planner inputs.
- Assuming a launch reference proves an ignored map exists on the deployment
  host.
