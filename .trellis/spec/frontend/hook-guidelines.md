# Hook Guidelines

> Hook guidance for the current repo state.

---

## Current Reality

There are no frontend hooks because there is no frontend framework in the repository.

There is no evidence of:

- React hooks
- Vue composables
- browser data-fetching hooks

## Rule

Do not invent hook patterns for this workspace.

If a task mentions reusable stateful logic today, it usually belongs to:

- a Python ROS node helper,
- a C++ utility or class,
- a launch argument pattern,
- or an RViz plugin implementation.
