#include "greenhouse_cmd_gate/command_gate_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace greenhouse_cmd_gate
{
namespace
{

double advance_toward(
  const double current,
  const double target,
  const double maximum_delta)
{
  return current + std::clamp(target - current, -maximum_delta, maximum_delta);
}

bool positive_finite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

CommandGateCore::CommandGateCore(GateLimits limits)
: limits_(limits)
{
  if (
    !positive_finite(limits_.max_linear_velocity) ||
    !positive_finite(limits_.max_angular_velocity) ||
    !positive_finite(limits_.max_linear_acceleration) ||
    !positive_finite(limits_.max_angular_acceleration) ||
    limits_.watchdog_timeout_ns <= 0 ||
    limits_.max_command_age_ns <= 0 ||
    limits_.future_tolerance_ns < 0)
  {
    throw std::invalid_argument("command-gate limits must be finite and positive");
  }
}

bool CommandGateCore::accept(
  const StampedCommand & command,
  const std::int64_t now_ns)
{
  if (watchdog_latched_ && !limits_.auto_recover_after_watchdog) {
    return false;
  }
  if (
    now_ns <= 0 ||
    command.stamp_ns <= 0 ||
    !std::isfinite(command.linear_velocity) ||
    !std::isfinite(command.angular_velocity))
  {
    force_zero(GateState::kInvalidCommand);
    return false;
  }
  const bool stamp_is_too_far_in_future =
    command.stamp_ns > now_ns &&
    command.stamp_ns - now_ns > limits_.future_tolerance_ns;
  const bool stamp_is_too_old =
    command.stamp_ns <= now_ns &&
    now_ns - command.stamp_ns > limits_.max_command_age_ns;
  if (
    stamp_is_too_far_in_future ||
    stamp_is_too_old)
  {
    force_zero(GateState::kInvalidCommand);
    return false;
  }

  target_linear_velocity_ = std::clamp(
    command.linear_velocity,
    -limits_.max_linear_velocity,
    limits_.max_linear_velocity);
  target_angular_velocity_ = std::clamp(
    command.angular_velocity,
    -limits_.max_angular_velocity,
    limits_.max_angular_velocity);
  last_receive_ns_ = now_ns;
  watchdog_latched_ = false;
  state_ = GateState::kActive;
  return true;
}

void CommandGateCore::reject_duplicate_source()
{
  force_zero(GateState::kDuplicateSource);
}

GateOutput CommandGateCore::update(const std::int64_t now_ns)
{
  if (now_ns <= 0) {
    force_zero(GateState::kInvalidCommand);
    return {
      output_linear_velocity_,
      output_angular_velocity_,
      state_,
    };
  }
  if (last_update_ns_.has_value() && now_ns < *last_update_ns_) {
    force_zero(GateState::kClockRegression);
    return {
      output_linear_velocity_,
      output_angular_velocity_,
      state_,
    };
  }
  if (
    last_receive_ns_.has_value() &&
    now_ns < *last_receive_ns_)
  {
    force_zero(GateState::kClockRegression);
    return {
      output_linear_velocity_,
      output_angular_velocity_,
      state_,
    };
  }
  if (
    last_receive_ns_.has_value() &&
    now_ns - *last_receive_ns_ > limits_.watchdog_timeout_ns)
  {
    watchdog_latched_ = true;
    force_zero(GateState::kWatchdogLatched);
  }

  if (state_ == GateState::kActive) {
    if (!last_update_ns_.has_value()) {
      last_update_ns_ = now_ns;
    }
    const double elapsed_seconds =
      static_cast<double>(now_ns - *last_update_ns_) / 1'000'000'000.0;
    output_linear_velocity_ = advance_toward(
      output_linear_velocity_,
      target_linear_velocity_,
      limits_.max_linear_acceleration * elapsed_seconds);
    output_angular_velocity_ = advance_toward(
      output_angular_velocity_,
      target_angular_velocity_,
      limits_.max_angular_acceleration * elapsed_seconds);
  } else {
    output_linear_velocity_ = 0.0;
    output_angular_velocity_ = 0.0;
  }

  last_update_ns_ = now_ns;
  return {
    output_linear_velocity_,
    output_angular_velocity_,
    state_,
  };
}

GateState CommandGateCore::state() const noexcept
{
  return state_;
}

bool CommandGateCore::watchdog_latched() const noexcept
{
  return watchdog_latched_;
}

std::optional<std::int64_t> CommandGateCore::last_receive_ns() const noexcept
{
  return last_receive_ns_;
}

void CommandGateCore::force_zero(const GateState state)
{
  state_ = state;
  target_linear_velocity_ = 0.0;
  target_angular_velocity_ = 0.0;
  output_linear_velocity_ = 0.0;
  output_angular_velocity_ = 0.0;
}

std::string_view to_string(const GateState state) noexcept
{
  switch (state) {
    case GateState::kStartupNoCommand:
      return "STARTUP_NO_COMMAND";
    case GateState::kActive:
      return "ACTIVE";
    case GateState::kInvalidCommand:
      return "INVALID_COMMAND";
    case GateState::kDuplicateSource:
      return "DUPLICATE_SOURCE";
    case GateState::kWatchdogLatched:
      return "WATCHDOG_LATCHED";
    case GateState::kClockRegression:
      return "CLOCK_REGRESSION";
  }
  return "UNKNOWN";
}

}  // namespace greenhouse_cmd_gate
