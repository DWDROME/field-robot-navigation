# Frontend Development Guidelines

> Frontend status for this workspace.

---

## Scope

There is no JavaScript or TypeScript frontend application in the repository as of July 26, 2026.

Evidence:

- no `package.json`
- no `tsconfig.json`
- no `.ts`, `.tsx`, `.js`, or `.jsx` application tree
- the operator-facing surfaces are ROS launch files, RViz configs, and a few RViz plugin packages under `src/MoveBase3D/rviz_plugins/`

For most tasks in this workspace, the frontend spec is effectively "not applicable".

## Pre-Development Checklist

If a task sounds UI-like, classify it first:

1. RViz config change:
   - treat it as workspace/operator tooling, not a web frontend
2. RViz plugin package change:
   - treat it as a C++ catkin package and follow backend rules
3. A future real web app:
   - create a dedicated app directory and update this spec before broad implementation

## Guides

| Guide | Current status |
|-------|----------------|
| [Directory Structure](./directory-structure.md) | N/A for current repo |
| [Component Guidelines](./component-guidelines.md) | N/A for current repo |
| [Hook Guidelines](./hook-guidelines.md) | N/A for current repo |
| [State Management](./state-management.md) | N/A for current repo |
| [Type Safety](./type-safety.md) | N/A for current repo |
| [Quality Guidelines](./quality-guidelines.md) | N/A for current repo |

## Practical Rule

Do not fabricate React, Vue, or browser-app conventions for this repository. When a task touches operator-facing surfaces today, it almost always belongs to backend/package/launch/RViz work instead.
