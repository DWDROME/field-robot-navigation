# Type Safety

> Type guidance for the current repo state.

---

## Current Reality

There is no TypeScript codebase in this repository.

Type safety today comes from:

- C++ compile-time types inside catkin packages
- ROS message definitions in packages such as `src/visibility_graph_msg/` and `src/dl_msgs/`
- Python message imports and ROS runtime contracts

## Rule

- Do not add TypeScript-specific guidance until TypeScript exists here.
- When a task changes shared data shape, prefer updating the ROS message definition or the owning Python/C++ contract instead of inventing a pseudo-type layer elsewhere.

## Examples

- `src/visibility_graph_msg/msg/`
- `src/dl_msgs/msg/`
- `src/dl_msgs/srv/`
