# State Management

> State guidance for the current repo state.

---

## Current Reality

There is no browser-side state management layer in this repository.

State is managed through robotics-native mechanisms instead:

- ROS topics
- TF
- ROS params
- launch arguments
- node-local C++ or Python memory

Examples:

- `src/greenhouse_nav/scripts/localization_manager.py` combines ROS params, subscribed odometry, and TF broadcasting
- `src/greenhouse_nav/launch/system_greenhouse_base.launch` wires launch arguments and ROS params into multiple nodes

## Rule

Do not describe Redux, Zustand, React Query, or similar tools here unless such a frontend is actually added.
