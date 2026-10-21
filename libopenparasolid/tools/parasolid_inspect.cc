// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <charconv>
#include <filesystem>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>

#include "openparasolid/parser.h"
#include "openparasolid/version.h"
#include "schema_report.h"

namespace {
using nlohmann::json;

std::string Hex(std::span<const uint8_t> bytes) {
  constexpr std::string_view digits = "0123456789abcdef";
  std::string result;
  result.reserve(bytes.size() * 2u);
  for (uint8_t byte : bytes) {
    result.push_back(digits[byte >> 4u]);
    result.push_back(digits[byte & 15u]);
  }
  return result;
}

json Report(const openparasolid::StreamInspection &inspection,
            const std::filesystem::path &source, bool summary) {
  const auto &h = inspection.header;
  json header{{"kind", openparasolid::ToString(h.kind)},
              {"description", h.description},
              {"schema", h.schema},
              {"description_offset", h.description_offset},
              {"schema_offset", h.schema_offset},
              {"schema_end_offset", h.schema_end_offset},
              {"body_offset", h.body_offset},
              {"internal_header_complete", h.internal_header_complete},
              {"schema_max_type",
               h.schema_max_type ? json(*h.schema_max_type) : json(nullptr)},
              {"user_field_words",
               h.user_field_words ? json(*h.user_field_words) : json(nullptr)},
              {"schema_version", nullptr}};
  if (h.schema_version)
    header["schema_version"] = {
        {"modeller", h.schema_version->modeller},
        {"schema", h.schema_version->schema},
        {"base_schema", h.schema_version->base_schema
                            ? json(*h.schema_version->base_schema)
                            : json(nullptr)}};
  std::map<std::string, uint64_t> kinds, references;
  uint64_t reference_count = 0;
  for (const auto &record : inspection.candidates) {
    ++kinds[openparasolid::ToString(record.kind)];
    for (const auto &reference : record.references) {
      ++references[openparasolid::ToString(reference.status)];
      ++reference_count;
    }
  }
  json report{{"report_version", 1},
              {"library_version", OPENPARASOLID_VERSION_STRING},
              {"source_file", source.string()},
              {"format", "binary_parasolid"},
              {"byte_size", inspection.byte_size},
              {"header", std::move(header)},
              {"interpretation", "fixed_layout_candidates"},
              {"record_scan_performed", inspection.record_scan_performed},
              {"record_scan_complete", inspection.record_scan_complete},
              {"record_candidate_count", inspection.candidates.size()},
              {"candidate_counts", kinds},
              {"reference_count", reference_count},
              {"reference_status_counts", references},
              {"candidate_prefix_bytes", inspection.candidate_prefix_bytes},
              {"uninterpreted_bytes", inspection.uninterpreted_bytes},
              {"unknown_region_count", inspection.unknown_regions.size()},
              {"notes", inspection.notes}};
  if (summary) return report;
  report["candidates"] = json::array();
  for (std::size_t i = 0; i < inspection.candidates.size(); ++i) {
    const auto &candidate = inspection.candidates[i];
    json item{{"index", i},
              {"offset", candidate.offset},
              {"prefix_size", candidate.prefix_size},
              {"tag", candidate.tag},
              {"kind", openparasolid::ToString(candidate.kind)},
              {"attribute", candidate.attribute},
              {"has_schema_marker", candidate.has_schema_marker},
              {"sequence",
               candidate.sequence ? json(*candidate.sequence) : json(nullptr)},
              {"sense", candidate.sense ? json(std::string(1, *candidate.sense))
                                        : json(nullptr)},
              {"position",
               candidate.position ? json(*candidate.position) : json(nullptr)},
              {"references", json::array()}};
    for (const auto &reference : candidate.references) {
      item["references"].push_back(
          {{"field", reference.field},
           {"offset", reference.offset},
           {"attribute", reference.attribute},
           {"expected_kind",
            reference.expected_kind
                ? json(openparasolid::ToString(*reference.expected_kind))
                : json(nullptr)},
           {"status", openparasolid::ToString(reference.status)},
           {"target_candidate", reference.target_candidate
                                    ? json(*reference.target_candidate)
                                    : json(nullptr)}});
    }
    report["candidates"].push_back(std::move(item));
  }
  report["unknown_regions"] = json::array();
  for (const auto &region : inspection.unknown_regions)
    report["unknown_regions"].push_back({{"offset", region.offset},
                                         {"size", region.size},
                                         {"preview_hex", Hex(region.preview)}});
  return report;
}

void Usage(std::ostream &output) {
  output << "Usage: parasolid_inspect [--summary] [--max-records N] "
            "[--nodes] [--schema-catalog FILE.json] [--max-fields N] "
            "[--max-values N] [--max-delta-entries N] <file.x_b|file.bin>\n"
            "Schema limits require --nodes or --schema-catalog.\n";
}

}  // namespace

int main(int argc, char *argv[]) {
  openparasolid::InspectOptions options;
  std::optional<std::filesystem::path> input;
  bool summary = false;
  bool nodes = false;
  openparasolid::SchemaLimits limits;
  bool schema_limits = false;
  std::optional<std::filesystem::path> catalog_path;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      Usage(std::cout);
      return 0;
    }
    if (arg == "--version") {
      std::cout << OPENPARASOLID_VERSION_STRING << '\n';
      return 0;
    }
    if (arg == "--summary")
      summary = true;
    else if (arg == "--nodes")
      nodes = true;
    else if (arg == "--schema-catalog") {
      if (++i >= argc) {
        Usage(std::cerr);
        return 1;
      }
      catalog_path = argv[i];
      nodes = true;
    } else if (arg == "--max-records" || arg == "--max-fields" ||
               arg == "--max-values" || arg == "--max-delta-entries") {
      if (++i >= argc) {
        Usage(std::cerr);
        return 1;
      }
      const std::string_view number = argv[i];
      uint64_t *target = &options.max_records;
      if (arg != "--max-records") {
        schema_limits = true;
        target = arg == "--max-fields"   ? &limits.max_total_fields
                 : arg == "--max-values" ? &limits.max_values
                                         : &limits.max_delta_entries;
      }
      const auto parsed = std::from_chars(
          number.data(), number.data() + number.size(), *target);
      if (number.empty() || parsed.ec != std::errc{} ||
          parsed.ptr != number.data() + number.size() || *target == 0) {
        Usage(std::cerr);
        return 1;
      }
    } else if (arg.starts_with('-') || input) {
      Usage(std::cerr);
      return 1;
    } else
      input = std::filesystem::path(arg);
  }
  if (!input) {
    Usage(std::cerr);
    return 1;
  }
  if (schema_limits && !nodes) {
    std::cerr << "Error: schema limits require --nodes or --schema-catalog\n";
    return 1;
  }
  if (nodes) {
    try {
      std::optional<openparasolid::SchemaCatalog> catalog;
      if (catalog_path)
        catalog = openparasolid::cli::LoadCatalog(*catalog_path);
      limits.max_nodes = options.max_records;
      const auto parsed = openparasolid::SchemaParser::ParseFile(
          *input, catalog ? &*catalog : nullptr, limits);
      if (!parsed.ok()) {
        const auto &error = parsed.error();
        std::cerr << "Error: " << openparasolid::ToString(error.code)
                  << " at byte " << error.offset << ": " << error.message
                  << '\n';
        return 1;
      }
      auto report =
          openparasolid::cli::SchemaReport(parsed.value(), *input, summary);
      report["limits"] = {{"max_input_bytes", limits.max_input_bytes},
                          {"max_nodes", limits.max_nodes},
                          {"max_types", limits.max_types},
                          {"max_fields_per_type", limits.max_fields_per_type},
                          {"max_total_fields", limits.max_total_fields},
                          {"max_values", limits.max_values},
                          {"max_delta_packets", limits.max_delta_packets},
                          {"max_delta_entries", limits.max_delta_entries}};
      std::cout << report.dump(2) << '\n';
      return parsed.value().complete ? 0 : 2;
    } catch (const std::exception &error) {
      std::cerr << "Error: " << error.what() << '\n';
      return 1;
    }
  }
  const auto result =
      openparasolid::ParasolidInspector::InspectFile(*input, options);
  if (!result.ok()) {
    const auto &error = result.error();
    std::cerr << "Error: " << openparasolid::ToString(error.code) << " at byte "
              << error.offset << ": " << error.message << '\n';
    return 1;
  }
  std::cout << Report(result.value(), *input, summary).dump(2) << '\n';
  return result.value().record_scan_complete ? 0 : 2;
}
