#include "dlrobot_robot/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dlrobot_robot
{
namespace
{

std::uint8_t XorBytes(const std::uint8_t * data, std::size_t size)
{
  std::uint8_t result = 0;
  for (std::size_t index = 0; index < size; ++index) {
    result ^= data[index];
  }
  return result;
}

std::int16_t ReadInt16(std::uint8_t high, std::uint8_t low)
{
  const auto value = static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(high) << 8U) | static_cast<std::uint16_t>(low));
  return static_cast<std::int16_t>(value);
}

std::int16_t ScaleCommand(double value)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument("velocity command must be finite");
  }

  const double scaled = value * 1000.0;
  const double limited = std::clamp(
    scaled,
    static_cast<double>(std::numeric_limits<std::int16_t>::min()),
    static_cast<double>(std::numeric_limits<std::int16_t>::max()));
  return static_cast<std::int16_t>(limited);
}

void WriteInt16(CommandFrame & frame, std::size_t offset, std::int16_t value)
{
  const auto raw = static_cast<std::uint16_t>(value);
  frame[offset] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
  frame[offset + 1] = static_cast<std::uint8_t>(raw & 0xFFU);
}

}  // namespace

CommandFrame EncodeVelocityCommand(double linear_x, double linear_y, double angular_z)
{
  CommandFrame frame{};
  frame[0] = kFrameHeader;
  WriteInt16(frame, 3, ScaleCommand(linear_x));
  WriteInt16(frame, 5, ScaleCommand(linear_y));
  WriteInt16(frame, 7, ScaleCommand(angular_z));
  frame[9] = XorBytes(frame.data(), 9);
  frame[10] = kFrameTail;
  return frame;
}

std::optional<Telemetry> DecodeTelemetry(const TelemetryFrame & frame)
{
  if (frame.front() != kFrameHeader || frame.back() != kFrameTail) {
    return std::nullopt;
  }
  if (frame[22] != XorBytes(frame.data(), 22)) {
    return std::nullopt;
  }

  Telemetry telemetry;
  telemetry.stop_flag = frame[1];
  telemetry.velocity_x = static_cast<float>(ReadInt16(frame[2], frame[3])) / 1000.0F;
  telemetry.velocity_y = static_cast<float>(ReadInt16(frame[4], frame[5])) / 1000.0F;
  telemetry.velocity_z = static_cast<float>(ReadInt16(frame[6], frame[7])) / 1000.0F;
  telemetry.acceleration_x =
    static_cast<float>(ReadInt16(frame[8], frame[9])) / kAccelerometerRatio;
  telemetry.acceleration_y =
    static_cast<float>(ReadInt16(frame[10], frame[11])) / kAccelerometerRatio;
  telemetry.acceleration_z =
    static_cast<float>(ReadInt16(frame[12], frame[13])) / kAccelerometerRatio;
  telemetry.angular_velocity_x =
    static_cast<float>(ReadInt16(frame[14], frame[15])) * kGyroscopeRatio;
  telemetry.angular_velocity_y =
    static_cast<float>(ReadInt16(frame[16], frame[17])) * kGyroscopeRatio;
  telemetry.angular_velocity_z =
    static_cast<float>(ReadInt16(frame[18], frame[19])) * kGyroscopeRatio;
  telemetry.voltage = static_cast<float>(ReadInt16(frame[20], frame[21])) / 1000.0F;
  return telemetry;
}

std::optional<Telemetry> TelemetryParser::Push(std::uint8_t byte)
{
  if (size_ == 0 && byte != kFrameHeader) {
    return std::nullopt;
  }

  buffer_[size_] = byte;
  ++size_;
  if (size_ < kTelemetryFrameSize) {
    return std::nullopt;
  }

  const auto telemetry = DecodeTelemetry(buffer_);
  if (telemetry.has_value()) {
    size_ = 0;
    return telemetry;
  }

  Resynchronize();
  return std::nullopt;
}

void TelemetryParser::Resynchronize()
{
  const auto next_header = std::find(buffer_.begin() + 1, buffer_.end(), kFrameHeader);
  if (next_header == buffer_.end()) {
    size_ = 0;
    return;
  }

  const auto retained = static_cast<std::size_t>(buffer_.end() - next_header);
  std::copy(next_header, buffer_.end(), buffer_.begin());
  size_ = retained;
}

}  // namespace dlrobot_robot
