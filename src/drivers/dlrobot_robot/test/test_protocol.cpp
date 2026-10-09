#include "dlrobot_robot/protocol.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

namespace
{

void Require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void WriteInt16(
  dlrobot_robot::TelemetryFrame & frame,
  std::size_t offset,
  std::int16_t value)
{
  const auto raw = static_cast<std::uint16_t>(value);
  frame[offset] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
  frame[offset + 1] = static_cast<std::uint8_t>(raw & 0xFFU);
}

std::uint8_t XorBytes(const dlrobot_robot::TelemetryFrame & frame, std::size_t size)
{
  std::uint8_t result = 0;
  for (std::size_t index = 0; index < size; ++index) {
    result ^= frame[index];
  }
  return result;
}

dlrobot_robot::TelemetryFrame MakeTelemetryFrame()
{
  dlrobot_robot::TelemetryFrame frame{};
  frame[0] = dlrobot_robot::kFrameHeader;
  frame[1] = 1;
  WriteInt16(frame, 2, 1250);
  WriteInt16(frame, 4, -500);
  WriteInt16(frame, 6, 750);
  WriteInt16(frame, 8, 1672);
  WriteInt16(frame, 10, -836);
  WriteInt16(frame, 12, 0);
  WriteInt16(frame, 14, 100);
  WriteInt16(frame, 16, -200);
  WriteInt16(frame, 18, 300);
  WriteInt16(frame, 20, 24500);
  frame[22] = XorBytes(frame, 22);
  frame[23] = dlrobot_robot::kFrameTail;
  return frame;
}

void TestCommandEncoding()
{
  const auto frame = dlrobot_robot::EncodeVelocityCommand(1.25, -0.5, 0.75);
  Require(frame[0] == dlrobot_robot::kFrameHeader, "command header");
  Require(frame[3] == 0x04 && frame[4] == 0xE2, "command linear x");
  Require(frame[5] == 0xFE && frame[6] == 0x0C, "command linear y");
  Require(frame[7] == 0x02 && frame[8] == 0xEE, "command angular z");
  Require(frame[10] == dlrobot_robot::kFrameTail, "command tail");

  std::uint8_t checksum = 0;
  for (std::size_t index = 0; index < 9; ++index) {
    checksum ^= frame[index];
  }
  Require(frame[9] == checksum, "command checksum");
}

void TestCommandSaturation()
{
  const auto frame = dlrobot_robot::EncodeVelocityCommand(100.0, -100.0, 0.0);
  Require(frame[3] == 0x7F && frame[4] == 0xFF, "positive command saturation");
  Require(frame[5] == 0x80 && frame[6] == 0x00, "negative command saturation");
}

void TestTelemetryDecoding()
{
  const auto telemetry = dlrobot_robot::DecodeTelemetry(MakeTelemetryFrame());
  Require(telemetry.has_value(), "valid telemetry");
  Require(std::abs(telemetry->velocity_x - 1.25F) < 1e-6F, "telemetry velocity x");
  Require(std::abs(telemetry->velocity_y + 0.5F) < 1e-6F, "telemetry velocity y");
  Require(std::abs(telemetry->voltage - 24.5F) < 1e-6F, "telemetry voltage");
}

void TestInvalidTelemetry()
{
  auto bad_checksum = MakeTelemetryFrame();
  bad_checksum[22] ^= 0x01;
  Require(!dlrobot_robot::DecodeTelemetry(bad_checksum).has_value(), "bad telemetry checksum");

  auto bad_tail = MakeTelemetryFrame();
  bad_tail[23] = 0;
  Require(!dlrobot_robot::DecodeTelemetry(bad_tail).has_value(), "bad telemetry tail");
}

void TestParserResynchronization()
{
  dlrobot_robot::TelemetryParser parser;

  auto malformed = MakeTelemetryFrame();
  malformed[22] ^= 0x01;
  malformed[10] = dlrobot_robot::kFrameHeader;
  for (const auto byte : malformed) {
    Require(!parser.Push(byte).has_value(), "malformed frame must not produce telemetry");
  }

  std::optional<dlrobot_robot::Telemetry> decoded;
  for (const auto byte : MakeTelemetryFrame()) {
    const auto telemetry = parser.Push(byte);
    if (telemetry.has_value()) {
      decoded = telemetry;
    }
  }
  Require(decoded.has_value(), "parser resynchronization");
}

}  // namespace

int main()
{
  try {
    TestCommandEncoding();
    TestCommandSaturation();
    TestTelemetryDecoding();
    TestInvalidTelemetry();
    TestParserResynchronization();
  } catch (const std::exception & error) {
    std::cerr << "protocol test failed: " << error.what() << '\n';
    return 1;
  }

  std::cout << "protocol tests passed\n";
  return 0;
}
