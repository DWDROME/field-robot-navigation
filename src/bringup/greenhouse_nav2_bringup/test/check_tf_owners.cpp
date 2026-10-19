#include <rmw/rmw.h>
#include <rmw/types.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

using Clock = std::chrono::steady_clock;
using TFMessage = tf2_msgs::msg::TFMessage;

constexpr std::array<const char *, 3> kRequiredEdges = {
  "map->odom",
  "odom->base_link",
  "base_link->livox_frame",
};

struct Arguments
{
  double duration_sec{2.0};
  double timeout_sec{60.0};
  std::string output;
};

struct Observation
{
  std::map<std::string, std::set<std::string>> owner_gids;
  std::map<std::string, std::uint64_t> sample_counts;
  std::set<std::string> observed_edges;
};

double parse_positive_double(const std::string & value, const char * name)
{
  std::size_t consumed = 0;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || parsed <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
  return parsed;
}

Arguments parse_arguments(int argc, char ** argv)
{
  Arguments arguments;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value for " + argument);
    }
    const std::string value = argv[++index];
    if (argument == "--duration-sec") {
      arguments.duration_sec = parse_positive_double(value, "--duration-sec");
    } else if (argument == "--timeout-sec") {
      arguments.timeout_sec = parse_positive_double(value, "--timeout-sec");
    } else if (argument == "--output") {
      arguments.output = value;
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (arguments.output.empty()) {
    throw std::invalid_argument("--output is required");
  }
  return arguments;
}

std::string normalized_frame(const std::string & frame)
{
  const auto first = frame.find_first_not_of('/');
  return first == std::string::npos ? std::string{} : frame.substr(first);
}

std::string transform_edge(const std::string & parent, const std::string & child)
{
  return normalized_frame(parent) + "->" + normalized_frame(child);
}

std::string publisher_gid(const rclcpp::MessageInfo & message_info)
{
  const rmw_gid_t & gid = message_info.get_rmw_message_info().publisher_gid;
  if (gid.implementation_identifier == nullptr) {
    return "unknown";
  }

  const bool all_zero = std::all_of(
    std::begin(gid.data),
    std::end(gid.data),
    [](std::uint8_t value) {return value == 0;});
  if (all_zero) {
    return "unknown";
  }

  std::ostringstream result;
  result << gid.implementation_identifier << ':';
  result << std::hex << std::setfill('0');
  for (const std::uint8_t value : gid.data) {
    result << std::setw(2) << static_cast<unsigned int>(value);
  }
  return result.str();
}

bool all_required_edges_observed(const Observation & observation)
{
  return std::all_of(
    kRequiredEdges.begin(),
    kRequiredEdges.end(),
    [&observation](const char * edge) {
      const auto owners = observation.owner_gids.find(edge);
      const auto samples = observation.sample_counts.find(edge);
      return owners != observation.owner_gids.end() &&
             !owners->second.empty() &&
             samples != observation.sample_counts.end() &&
             samples->second > 0;
    });
}

bool ownership_is_unique(const Observation & observation)
{
  return std::all_of(
    kRequiredEdges.begin(),
    kRequiredEdges.end(),
    [&observation](const char * edge) {
      const auto owners = observation.owner_gids.find(edge);
      return owners != observation.owner_gids.end() &&
             owners->second.size() == 1 &&
             owners->second.count("unknown") == 0;
    });
}

void write_json(
  const std::string & output,
  const Observation & observation,
  bool timed_out)
{
  std::ofstream stream(output, std::ios::out | std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("cannot open output: " + output);
  }

  const bool all_observed = all_required_edges_observed(observation);
  const bool passed = all_observed && ownership_is_unique(observation) && !timed_out;
  stream << "{\n";
  stream << "  \"schema_version\": 1,\n";
  stream << "  \"passed\": " << (passed ? "true" : "false") << ",\n";
  stream << "  \"timed_out\": " << (timed_out ? "true" : "false") << ",\n";
  stream << "  \"rmw_identifier\": \""
         << rmw_get_implementation_identifier() << "\",\n";
  stream << "  \"required_edges\": [\n";
  for (std::size_t index = 0; index < kRequiredEdges.size(); ++index) {
    stream << "    \"" << kRequiredEdges[index] << "\""
           << (index + 1 == kRequiredEdges.size() ? "\n" : ",\n");
  }
  stream << "  ],\n";
  stream << "  \"edges\": {\n";
  for (std::size_t index = 0; index < kRequiredEdges.size(); ++index) {
    const std::string edge = kRequiredEdges[index];
    const auto owners = observation.owner_gids.find(edge);
    const auto samples = observation.sample_counts.find(edge);
    const std::size_t owner_count =
      owners == observation.owner_gids.end() ? 0 : owners->second.size();
    const std::uint64_t sample_count =
      samples == observation.sample_counts.end() ? 0 : samples->second;

    stream << "    \"" << edge << "\": {\n";
    stream << "      \"owner_count\": " << owner_count << ",\n";
    stream << "      \"owner_gid\": [";
    if (owners != observation.owner_gids.end()) {
      std::size_t owner_index = 0;
      for (const std::string & owner : owners->second) {
        stream << (owner_index++ == 0 ? "" : ", ") << "\"" << owner << "\"";
      }
    }
    stream << "],\n";
    stream << "      \"sample_count\": " << sample_count << "\n";
    stream << "    }"
           << (index + 1 == kRequiredEdges.size() ? "\n" : ",\n");
  }
  stream << "  },\n";
  stream << "  \"observed_edges\": [";
  std::size_t observed_index = 0;
  for (const std::string & edge : observation.observed_edges) {
    stream << (observed_index++ == 0 ? "" : ", ") << "\"" << edge << "\"";
  }
  stream << "]\n";
  stream << "}\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Arguments arguments = parse_arguments(argc, argv);
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("greenhouse_tf_owner_probe");
    Observation observation;

    const auto callback =
      [&observation](TFMessage::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        const std::string gid = publisher_gid(info);
        for (const auto & transform : message->transforms) {
          const std::string edge = transform_edge(
            transform.header.frame_id,
            transform.child_frame_id);
          observation.owner_gids[edge].insert(gid);
          ++observation.sample_counts[edge];
          observation.observed_edges.insert(edge);
        }
      };

    const auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(100))
      .best_effort()
      .durability_volatile();
    const auto tf_static_qos = rclcpp::QoS(rclcpp::KeepLast(100))
      .reliable()
      .transient_local();
    auto tf_subscription =
      node->create_subscription<TFMessage>("/tf", tf_qos, callback);
    auto tf_static_subscription =
      node->create_subscription<TFMessage>("/tf_static", tf_static_qos, callback);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto start = Clock::now();
    const auto timeout = std::chrono::duration<double>(arguments.timeout_sec);
    const auto observation_duration =
      std::chrono::duration<double>(arguments.duration_sec);
    Clock::time_point ready_at{};
    bool ready = false;

    while (rclcpp::ok() && Clock::now() - start < timeout) {
      executor.spin_some();
      const auto now = Clock::now();
      if (all_required_edges_observed(observation)) {
        if (!ready) {
          ready = true;
          ready_at = now;
        }
        if (now - ready_at >= observation_duration) {
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const bool timed_out = !ready ||
      Clock::now() - ready_at < observation_duration;
    write_json(arguments.output, observation, timed_out);
    const bool passed =
      !timed_out &&
      all_required_edges_observed(observation) &&
      ownership_is_unique(observation);

    executor.remove_node(node);
    tf_subscription.reset();
    tf_static_subscription.reset();
    node.reset();
    rclcpp::shutdown();
    return passed ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 2;
  }
}
