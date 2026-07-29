#ifndef DLROBOT_ROBOT__PROTOCOL_HPP_
#define DLROBOT_ROBOT__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace dlrobot_robot
{

constexpr std::uint8_t kFrameHeader = 0x7B;
constexpr std::uint8_t kFrameTail = 0x7D;
constexpr std::size_t kCommandFrameSize = 11;
constexpr std::size_t kTelemetryFrameSize = 24;
constexpr float kGyroscopeRatio = 0.00026644F;
constexpr float kAccelerometerRatio = 1671.84F;

using CommandFrame = std::array<std::uint8_t, kCommandFrameSize>;
using TelemetryFrame = std::array<std::uint8_t, kTelemetryFrameSize>;

struct Telemetry
{
  std::uint8_t stop_flag{0};
  float velocity_x{0.0F};
  float velocity_y{0.0F};
  float velocity_z{0.0F};
  float acceleration_x{0.0F};
  float acceleration_y{0.0F};
  float acceleration_z{0.0F};
  float angular_velocity_x{0.0F};
  float angular_velocity_y{0.0F};
  float angular_velocity_z{0.0F};
  float voltage{0.0F};
};

CommandFrame EncodeVelocityCommand(double linear_x, double linear_y, double angular_z);
std::optional<Telemetry> DecodeTelemetry(const TelemetryFrame & frame);

class TelemetryParser
{
public:
  std::optional<Telemetry> Push(std::uint8_t byte);

private:
  void Resynchronize();

  TelemetryFrame buffer_{};
  std::size_t size_{0};
};

}  // namespace dlrobot_robot

#endif  // DLROBOT_ROBOT__PROTOCOL_HPP_
