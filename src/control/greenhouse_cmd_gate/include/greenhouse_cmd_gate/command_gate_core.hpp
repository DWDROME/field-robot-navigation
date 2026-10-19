#ifndef GREENHOUSE_CMD_GATE__COMMAND_GATE_CORE_HPP_
#define GREENHOUSE_CMD_GATE__COMMAND_GATE_CORE_HPP_

#include <cstdint>
#include <optional>
#include <string_view>

namespace greenhouse_cmd_gate
{

enum class GateState
{
  kStartupNoCommand,
  kActive,
  kInvalidCommand,
  kDuplicateSource,
  kWatchdogLatched,
  kClockRegression,
};

struct GateLimits
{
  double max_linear_velocity;
  double max_angular_velocity;
  double max_linear_acceleration;
  double max_angular_acceleration;
  std::int64_t watchdog_timeout_ns;
  std::int64_t max_command_age_ns;
  std::int64_t future_tolerance_ns;
  bool auto_recover_after_watchdog;
};

struct StampedCommand
{
  double linear_velocity;
  double angular_velocity;
  std::int64_t stamp_ns;
};

struct GateOutput
{
  double linear_velocity;
  double angular_velocity;
  GateState state;
};

class CommandGateCore
{
public:
  explicit CommandGateCore(GateLimits limits);

  bool accept(const StampedCommand & command, std::int64_t now_ns);
  void reject_duplicate_source();
  GateOutput update(std::int64_t now_ns);

  [[nodiscard]] GateState state() const noexcept;
  [[nodiscard]] bool watchdog_latched() const noexcept;
  [[nodiscard]] std::optional<std::int64_t> last_receive_ns() const noexcept;

private:
  void force_zero(GateState state);

  GateLimits limits_;
  GateState state_{GateState::kStartupNoCommand};
  double target_linear_velocity_{0.0};
  double target_angular_velocity_{0.0};
  double output_linear_velocity_{0.0};
  double output_angular_velocity_{0.0};
  std::optional<std::int64_t> last_receive_ns_;
  std::optional<std::int64_t> last_update_ns_;
  bool watchdog_latched_{false};
};

[[nodiscard]] std::string_view to_string(GateState state) noexcept;

}  // namespace greenhouse_cmd_gate

#endif  // GREENHOUSE_CMD_GATE__COMMAND_GATE_CORE_HPP_
