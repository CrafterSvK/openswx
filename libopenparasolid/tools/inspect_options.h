// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_TOOLS_INSPECT_OPTIONS_H_
#define OPENPARASOLID_TOOLS_INSPECT_OPTIONS_H_

#include <charconv>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include "openparasolid/brep.h"
#include "openparasolid/parser.h"
#include "openparasolid/schema.h"

namespace openparasolid::cli {
// Limits mirror the default binding budgets and bound text processing before
// allocating interval output, independently of the input model's schema limits.
template <typename Interval>
Result<std::vector<Interval>> ParseIntervalSetting(std::string_view text) {
  constexpr std::size_t kMaxIntervals = 100000;
  constexpr std::size_t kMaxTextBytes = 16 * 1024 * 1024;
  auto invalid = [](const char* message) {
    return Result<std::vector<Interval>>::Err(
        {ErrorCode::kInvalidOptions, 0, message});
  };
  if (text.size() > kMaxTextBytes)
    return Result<std::vector<Interval>>::Err(
        {ErrorCode::kResourceLimit, 0, "Interval setting text limit"});
  std::vector<Interval> intervals;
  std::unordered_set<uint32_t> seen;
  while (!text.empty()) {
    if (intervals.size() == kMaxIntervals)
      return Result<std::vector<Interval>>::Err(
          {ErrorCode::kResourceLimit, 0, "Interval setting count limit"});
    const auto comma = text.find(',');
    const auto entry = text.substr(0, comma);
    const auto first = entry.find(':');
    const auto second =
        first == std::string_view::npos ? first : entry.find(':', first + 1);
    if (first == std::string_view::npos || second == std::string_view::npos ||
        entry.find(':', second + 1) != std::string_view::npos)
      return invalid(
          "Intervals require ID:START:END entries separated by commas");
    uint32_t id = 0;
    const auto identifier = entry.substr(0, first);
    const auto parsed_id = std::from_chars(
        identifier.data(), identifier.data() + identifier.size(), id);
    if (identifier.empty() || parsed_id.ec != std::errc{} ||
        parsed_id.ptr != identifier.data() + identifier.size() || !id ||
        !seen.insert(id).second)
      return invalid("Duplicate or invalid interval ID");
    auto number = [](std::string_view token) -> std::optional<double> {
      double value = 0;
      const auto parsed =
          std::from_chars(token.data(), token.data() + token.size(), value);
      if (token.empty() || parsed.ec != std::errc{} ||
          parsed.ptr != token.data() + token.size() || !std::isfinite(value))
        return std::nullopt;
      return value;
    };
    const auto start = number(entry.substr(first + 1, second - first - 1));
    const auto end = number(entry.substr(second + 1));
    if (!start || !end || *start == *end)
      return invalid("Intervals require finite distinct parameters");
    intervals.push_back({NodeId{id}, *start, *end});
    if (comma == std::string_view::npos) break;
    text.remove_prefix(comma + 1);
    if (text.empty()) return invalid("Empty interval after comma");
  }
  return Result<std::vector<Interval>>::Ok(std::move(intervals));
}
enum class InspectAction { kRun, kHelp, kVersion };
struct InspectSettings {
  InspectOptions inspect;
  SchemaLimits limits;
  std::optional<std::filesystem::path> input;
  std::optional<std::filesystem::path> catalog;
  std::optional<uint32_t> body_geometry;
  std::optional<uint32_t> body_topology;
  std::optional<double> binding_tolerance;
  std::optional<IntersectionEvaluationOptions> intersection;
  std::vector<FinParameterInterval> fin_intervals;
  std::vector<WireParameterInterval> wire_intervals;
  bool summary = false;
  bool nodes = false;
  bool geometry = false;
  bool boundary_geometry = false;
  InspectAction action = InspectAction::kRun;
};
using EnvironmentLookup =
    std::function<std::optional<std::string>(std::string_view)>;
inline Result<InspectSettings> ParseInspectSettings(
    std::span<const std::string_view> arguments,
    const EnvironmentLookup& environment) {
  if (!environment)
    return Result<InspectSettings>::Err(
        {ErrorCode::kInvalidOptions, 0, "Environment lookup is required"});
  InspectSettings settings;
  std::map<std::string, std::string> raw{
      {"summary", "0"},
      {"nodes", "0"},
      {"geometry", "0"},
      {"boundary-geometry", "0"},
      {"body-geometry", ""},
      {"body-topology", ""},
      {"fin-intervals", ""},
      {"wire-intervals", ""},
      {"binding-tolerance", ""},
      {"intersection-chart-tolerance", ""},
      {"intersection-solve-tolerance", ""},
      {"schema-catalog", ""},
      {"max-records", std::to_string(settings.inspect.max_records)},
      {"max-fields", std::to_string(settings.limits.max_total_fields)},
      {"max-values", std::to_string(settings.limits.max_values)},
      {"max-delta-entries", std::to_string(settings.limits.max_delta_entries)}};
  constexpr std::pair<const char*, const char*> names[]{
      {"summary", "OPENPARASOLID_SUMMARY"},
      {"nodes", "OPENPARASOLID_NODES"},
      {"geometry", "OPENPARASOLID_GEOMETRY"},
      {"boundary-geometry", "OPENPARASOLID_BOUNDARY_GEOMETRY"},
      {"body-geometry", "OPENPARASOLID_BODY_GEOMETRY"},
      {"body-topology", "OPENPARASOLID_BODY_TOPOLOGY"},
      {"fin-intervals", "OPENPARASOLID_FIN_INTERVALS"},
      {"wire-intervals", "OPENPARASOLID_WIRE_INTERVALS"},
      {"binding-tolerance", "OPENPARASOLID_BINDING_TOLERANCE"},
      {"intersection-chart-tolerance",
       "OPENPARASOLID_INTERSECTION_CHART_TOLERANCE"},
      {"intersection-solve-tolerance",
       "OPENPARASOLID_INTERSECTION_SOLVE_TOLERANCE"},
      {"schema-catalog", "OPENPARASOLID_SCHEMA_CATALOG"},
      {"max-records", "OPENPARASOLID_MAX_RECORDS"},
      {"max-fields", "OPENPARASOLID_MAX_FIELDS"},
      {"max-values", "OPENPARASOLID_MAX_VALUES"},
      {"max-delta-entries", "OPENPARASOLID_MAX_DELTA_ENTRIES"}};
  bool schema_limits = false;
  for (const auto& [option, variable] : names) {
    if (auto value = environment(variable)) {
      raw[option] = std::move(*value);
      if (std::string_view(option) == "max-fields" ||
          std::string_view(option) == "max-values" ||
          std::string_view(option) == "max-delta-entries")
        schema_limits = true;
    }
  }
  auto invalid = [](std::string message) {
    return Result<InspectSettings>::Err(
        {ErrorCode::kInvalidOptions, 0, std::move(message)});
  };
  for (std::size_t i = 0; i < arguments.size(); ++i) {
    const auto arg = arguments[i];
    if (arg == "--help" || arg == "-h" || arg == "--version") {
      settings.action =
          arg == "--version" ? InspectAction::kVersion : InspectAction::kHelp;
      return Result<InspectSettings>::Ok(std::move(settings));
    }
    if (!arg.starts_with('-')) {
      if (settings.input) return invalid("Only one input path is accepted");
      settings.input = std::filesystem::path(arg);
      continue;
    }
    if (!arg.starts_with("--"))
      return invalid("Unknown option: " + std::string(arg));
    const auto equal = arg.find('=');
    const auto name = std::string(
        arg.substr(2, equal == std::string_view::npos ? equal : equal - 2));
    if (!raw.contains(name))
      return invalid("Unknown option: " + std::string(arg));
    const bool boolean = name == "summary" || name == "nodes" ||
                         name == "geometry" || name == "boundary-geometry";
    if (equal != std::string_view::npos)
      raw[name] = arg.substr(equal + 1);
    else if (boolean)
      raw[name] = "1";
    else {
      if (++i == arguments.size())
        return invalid("Missing option value: " + name);
      raw[name] = arguments[i];
    }
    if (name == "max-fields" || name == "max-values" ||
        name == "max-delta-entries")
      schema_limits = true;
  }
  for (const auto& [name, target] :
       {std::pair{"summary", &settings.summary},
        {"nodes", &settings.nodes},
        {"geometry", &settings.geometry},
        {"boundary-geometry", &settings.boundary_geometry}}) {
    const auto& text = raw[name];
    if (text != "0" && text != "1" && text != "false" && text != "true")
      return invalid("Invalid boolean setting: " + std::string(name));
    *target = text == "1" || text == "true";
  }
  for (const auto& [name, target] :
       {std::pair{"max-records", &settings.inspect.max_records},
        {"max-fields", &settings.limits.max_total_fields},
        {"max-values", &settings.limits.max_values},
        {"max-delta-entries", &settings.limits.max_delta_entries}}) {
    const auto& text = raw[name];
    const auto parsed =
        std::from_chars(text.data(), text.data() + text.size(), *target);
    if (text.empty() || parsed.ec != std::errc{} ||
        parsed.ptr != text.data() + text.size() || *target == 0)
      return invalid("Invalid positive integer setting: " + std::string(name));
  }
  if (!raw["body-topology"].empty()) {
    uint32_t id = 0;
    const auto& text = raw["body-topology"];
    const auto parsed =
        std::from_chars(text.data(), text.data() + text.size(), id);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
        !id)
      return invalid("Invalid positive topology body ID");
    settings.body_topology = id;
    settings.geometry = true;
  }
  if (!raw["body-geometry"].empty()) {
    uint32_t id = 0;
    const auto& text = raw["body-geometry"];
    const auto parsed =
        std::from_chars(text.data(), text.data() + text.size(), id);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
        !id)
      return invalid("Invalid positive body ID");
    settings.body_geometry = id;
    settings.geometry = true;
    if (raw["binding-tolerance"].empty())
      return invalid("Body geometry requires --binding-tolerance");
  } else if (!raw["fin-intervals"].empty() || !raw["wire-intervals"].empty() ||
             !raw["binding-tolerance"].empty() ||
             !raw["intersection-chart-tolerance"].empty() ||
             !raw["intersection-solve-tolerance"].empty()) {
    return invalid("Binding settings require --body-geometry");
  }
  if (settings.boundary_geometry && !settings.body_geometry)
    return invalid("Boundary geometry requires --body-geometry");
  auto number = [&](const char* name, bool positive) -> Result<double> {
    double value = 0;
    const auto& text = raw[name];
    const auto parsed =
        std::from_chars(text.data(), text.data() + text.size(), value);
    if (text.empty() || parsed.ec != std::errc{} ||
        parsed.ptr != text.data() + text.size() || !std::isfinite(value) ||
        value < 0 || (positive && value == 0))
      return Result<double>::Err(
          {ErrorCode::kInvalidOptions, 0,
           "Invalid binding tolerance: " + std::string(name)});
    return Result<double>::Ok(value);
  };
  if (settings.body_geometry) {
    const auto tolerance = number("binding-tolerance", false);
    if (!tolerance.ok()) return invalid(tolerance.error().message);
    settings.binding_tolerance = tolerance.value();
    auto fins =
        ParseIntervalSetting<FinParameterInterval>(raw["fin-intervals"]);
    if (!fins.ok()) return Result<InspectSettings>::Err(fins.error());
    auto wires =
        ParseIntervalSetting<WireParameterInterval>(raw["wire-intervals"]);
    if (!wires.ok()) return Result<InspectSettings>::Err(wires.error());
    settings.fin_intervals = std::move(fins).value();
    settings.wire_intervals = std::move(wires).value();
    const bool chart = !raw["intersection-chart-tolerance"].empty();
    const bool solve = !raw["intersection-solve-tolerance"].empty();
    if (chart != solve)
      return invalid(
          "Intersection binding requires both chart and solve tolerances");
    if (chart) {
      const auto chart_tolerance =
          number("intersection-chart-tolerance", false);
      const auto solve_tolerance = number("intersection-solve-tolerance", true);
      if (!chart_tolerance.ok())
        return invalid(chart_tolerance.error().message);
      if (!solve_tolerance.ok())
        return invalid(solve_tolerance.error().message);
      settings.intersection = IntersectionEvaluationOptions{
          chart_tolerance.value(), solve_tolerance.value()};
    }
  }
  if (!raw["schema-catalog"].empty()) settings.catalog = raw["schema-catalog"];
  settings.nodes =
      settings.nodes || settings.geometry || settings.catalog.has_value();
  settings.limits.max_nodes = settings.inspect.max_records;
  if (schema_limits && !settings.nodes)
    return invalid(
        "Schema limits require --nodes, --geometry or --schema-catalog");
  if (!settings.input) return invalid("An input path is required");
  return Result<InspectSettings>::Ok(std::move(settings));
}
}  // namespace openparasolid::cli

#endif  // OPENPARASOLID_TOOLS_INSPECT_OPTIONS_H_
