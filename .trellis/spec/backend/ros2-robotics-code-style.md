# ROS 2 Robotics C++ Style

> Executable conventions for ROS 2 hardware-facing packages in this mixed ROS 1/ROS 2 repository.

## 1. Scope / Trigger

Apply this spec when creating or changing a ROS 2 C++ node, especially a serial driver, chassis interface, sensor adapter, or control boundary.

The first package using this contract is `src/dlrobot_robot`.

## 2. Signatures

ROS 2 packages use:

```text
package.xml: format 3, <build_type>ament_cmake</build_type>
CMake: ament_cmake, C++17
build: colcon build --packages-select <package>
launch: launch/*.launch.py
```

The `dlrobot_robot` interface is:

```text
cmd_vel       geometry_msgs/msg/TwistStamped  subscription
odom          nav_msgs/msg/Odometry           publisher
imu           sensor_msgs/msg/Imu             publisher
PowerVoltage  std_msgs/msg/Float32             publisher
```

## 3. Contracts

- Keep node, Topic, parameter, frame, and message names stable unless the task explicitly changes the public interface.
- Use relative Topic names in package code so launch files can namespace or remap them.
- Keep hardware protocols in ROS-independent functions or classes.
- Use fixed-size containers for fixed-size wire frames.
- Use RAII for hardware ownership. One serial device has exactly one process owner.
- Keep callbacks short: validate or convert input, call the protocol/hardware operation, and publish the result.
- Prefer fixed state, explicit members, early returns, and shallow control flow.
- Do not add global mutable state, hidden fallback, fake success, silent reconnect, or a second protocol implementation.
- Parameters that make the node unusable when invalid must fail startup explicitly.

## 4. Validation & Error Matrix

| Condition | Required behavior |
|-----------|-------------------|
| Invalid parameter such as non-positive baud rate | Throw during startup and exit non-zero |
| Serial device cannot open | Log the concrete device failure and exit non-zero |
| Partial or failed command write | Log `FATAL` and shut down |
| Serial read failure | Log `FATAL` and shut down |
| Incomplete input frame | Retain bounded parser state and wait for more bytes |
| Bad header, checksum, or tail | Reject the frame and resynchronize without publishing |
| Non-finite command value | Reject explicitly; do not encode an undefined integer |
| Command exceeds wire range | Clamp to the protocol integer range before encoding |

## 5. Good / Base / Bad Cases

- Good: a valid `TwistStamped` command becomes one protocol frame, and valid telemetry publishes all declared outputs.
- Base: noise or an incomplete frame produces no telemetry and does not overrun the fixed buffer.
- Bad: a missing serial device terminates startup with a clear error instead of leaving a healthy-looking node.

## 6. Tests Required

Protocol code must have ROS-independent tests that assert:

- command field positions, scaling, checksum, header, and tail;
- positive and negative command limits;
- telemetry field decoding and unit conversion;
- checksum and tail rejection;
- recovery from malformed input followed by a valid frame.

Package changes also require, when the environment is available:

```bash
colcon build --packages-select <package>
colcon test --packages-select <package>
python3 -m py_compile <package>/launch/*.launch.py
```

Hardware behavior remains unverified until tested on the target controller and vehicle.

## 7. Wrong vs Correct

### Wrong

```cpp
std::vector<uint8_t> frame;
try {
  serial.write(frame);
} catch (...) {
  // Continue as if the command was sent.
}
```

This hides a control failure, dynamically sizes a fixed protocol frame, and leaves serial ownership ambiguous.

### Correct

```cpp
const CommandFrame frame = EncodeVelocityCommand(linear_x, linear_y, angular_z);
const std::size_t written = serial.write(frame.data(), frame.size());
if (written != frame.size()) {
  throw std::runtime_error("incomplete serial command write");
}
```

The frame size, failure condition, and hardware result are explicit.
