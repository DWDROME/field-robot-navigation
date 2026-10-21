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

using CommandFrame = std::array<std::uint8_t, kCommandFrameSize>;
using TelemetryFrame = std::array<std::uint8_t, kTelemetryFrameSize>;

struct Telemetry
{
  std::uint8_t stop_flag{0};
  float velocity_x{0.0F};
  float velocity_y{0.0F};
  float velocity_z{0.0F};
  // 原样保留 8..19 字节的六个诊断值；不换算为 IMU。
  std::array<std::int16_t, 6> diagnostics{};
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
