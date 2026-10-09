#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "greenhouse_cmd_gate/command_gate_core.hpp"

namespace
{

using greenhouse_cmd_gate::CommandGateCore;
using greenhouse_cmd_gate::GateLimits;
using greenhouse_cmd_gate::GateState;
using greenhouse_cmd_gate::StampedCommand;

constexpr std::int64_t kSecond = 1'000'000'000;

void require(const bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void require_near(
  const double actual,
  const double expected,
  const double tolerance,
  const std::string & message)
{
  require(std::abs(actual - expected) <= tolerance, message);
}

GateLimits limits(const bool auto_recover)
{
  return {
    1.0,
    2.0,
    1.0,
    2.0,
    500'000'000,
    200'000'000,
    50'000'000,
    auto_recover,
  };
}

void test_startup_and_limits()
{
  CommandGateCore gate(limits(false));
  const auto startup = gate.update(kSecond);
  require(startup.state == GateState::kStartupNoCommand, "startup state");
  require_near(startup.linear_velocity, 0.0, 0.0, "startup linear zero");

  require(
    gate.accept({3.0, -4.0, kSecond}, kSecond),
    "fresh finite command should be accepted");
  gate.update(kSecond);
  const auto limited = gate.update(kSecond + 100'000'000);
  require_near(limited.linear_velocity, 0.1, 1e-12, "linear acceleration");
  require_near(limited.angular_velocity, -0.2, 1e-12, "angular acceleration");

  const auto saturated = gate.update(kSecond + 2 * kSecond);
  require_near(saturated.linear_velocity, 0.0, 0.0, "watchdog linear zero");
  require(
    saturated.state == GateState::kWatchdogLatched,
    "watchdog should latch");
  require(gate.watchdog_latched(), "watchdog latch flag");
  require(
    !gate.accept({0.1, 0.1, 3 * kSecond}, 3 * kSecond),
    "latched watchdog should reject recovery when disabled");
}

void test_invalid_and_duplicate_commands()
{
  CommandGateCore gate(limits(true));
  require(
    !gate.accept(
      {std::numeric_limits<double>::quiet_NaN(), 0.0, kSecond},
      kSecond),
    "NaN command must be rejected");
  require(
    gate.state() == GateState::kInvalidCommand,
    "NaN rejection state");
  require(
    !gate.accept({0.1, 0.1, kSecond}, kSecond + 300'000'000),
    "stale command must be rejected");
  gate.reject_duplicate_source();
  const auto duplicate = gate.update(kSecond + 400'000'000);
  require(
    duplicate.state == GateState::kDuplicateSource,
    "duplicate source state");
  require_near(duplicate.angular_velocity, 0.0, 0.0, "duplicate source zero");
}

void test_auto_recovery_fixture()
{
  CommandGateCore gate(limits(true));
  require(gate.accept({0.5, 0.5, kSecond}, kSecond), "initial command");
  gate.update(kSecond);
  const auto timed_out = gate.update(kSecond + 600'000'000);
  require(
    timed_out.state == GateState::kWatchdogLatched,
    "auto-recovery mode still records timeout");
  require(
    gate.accept(
      {0.25, -0.25, kSecond + 600'000'000},
      kSecond + 600'000'000),
    "valid command may clear latch only when explicitly configured");
  require(!gate.watchdog_latched(), "valid command clears configured latch");
}

void test_clock_regression_remains_fail_safe()
{
  CommandGateCore gate(limits(true));
  require(gate.accept({0.5, 0.0, 2 * kSecond}, 2 * kSecond), "command");
  gate.update(2 * kSecond);
  const auto regressed = gate.update(kSecond);
  require(
    regressed.state == GateState::kClockRegression,
    "clock regression state");
  require_near(regressed.linear_velocity, 0.0, 0.0, "regression forces zero");

  require(
    gate.accept({0.5, 0.0, kSecond}, kSecond),
    "fresh command may be received while the clock is regressed");
  const auto still_regressed = gate.update(kSecond + 100'000'000);
  require(
    still_regressed.state == GateState::kClockRegression,
    "old update baseline must not be rewound");
  require_near(
    still_regressed.linear_velocity,
    0.0,
    0.0,
    "regressed clock cannot bypass acceleration limits");
}

void test_future_tolerance()
{
  CommandGateCore gate(limits(true));
  require(
    gate.accept({0.1, 0.1, kSecond + 50'000'000}, kSecond),
    "boundary future tolerance should be accepted");
  require(
    !gate.accept({0.1, 0.1, kSecond + 50'000'001}, kSecond),
    "command beyond future tolerance should be rejected");
}

}  // namespace

int main()
{
  try {
    test_startup_and_limits();
    test_invalid_and_duplicate_commands();
    test_auto_recovery_fixture();
    test_clock_regression_remains_fail_safe();
    test_future_tolerance();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
