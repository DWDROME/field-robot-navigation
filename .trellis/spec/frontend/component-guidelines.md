# Component Guidelines

> Component guidance for the current repo state.

---

## Current Reality

There are no React, Vue, Svelte, or similar UI components in this workspace.

The only UI-adjacent code is:

- RViz plugin packages under `src/MoveBase3D/rviz_plugins/`
- RViz layout/config files such as `src/greenhouse_nav/rviz/greenhouse_mapping_base_link.rviz`

## Rule

- Do not write component conventions for frameworks that are not present.
- When touching RViz plugins, follow the owning catkin package conventions and backend quality rules.
- When touching `.rviz` files, keep changes scoped to the operator workflow being adjusted.

## Future Trigger

If a real frontend framework is introduced later, replace this file with framework-specific rules grounded in the actual app structure.
