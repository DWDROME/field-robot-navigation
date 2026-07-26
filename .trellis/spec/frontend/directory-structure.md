# Directory Structure

> Frontend directory guidance for the current repo state.

---

## Current Reality

There is no frontend application directory structure to document.

The nearest equivalents are:

- RViz configuration files, for example `src/greenhouse_nav/rviz/greenhouse_mapping_base_link.rviz`
- RViz plugin catkin packages under `src/MoveBase3D/rviz_plugins/`

These are not organized like a browser frontend and should not be documented as one.

## Rule

- Treat RViz plugin packages as ordinary catkin packages.
- Treat `.rviz` files as operator configuration artifacts.
- Future-only guidance: if a real web UI is added later, place it in a dedicated top-level app directory instead of mixing it into ROS package source trees.

## Examples

- `src/greenhouse_nav/rviz/`
- `src/MoveBase3D/rviz_plugins/rviz_goal_tool_plugins/`
- `src/MoveBase3D/rviz_plugins/rviz_multi_goals_manager_plugins/`
- `src/MoveBase3D/rviz_plugins/rviz_waypoint_generator_plugins/`
