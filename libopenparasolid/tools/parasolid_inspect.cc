// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "body_report.h"
#include "inspect_options.h"
#include "model_report.h"
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
  output
      << "Usage: parasolid_inspect [--summary] [--max-records N] "
         "[--nodes] [--geometry] [--schema-catalog FILE.json] [--max-fields N] "
         "[--max-values N] [--max-delta-entries N] "
         "[--body-geometry ID --binding-tolerance X] "
         "[--fin-intervals ID:START:END,...] [--wire-intervals ID:START:END,...] "
         "[--intersection-chart-tolerance X --intersection-solve-tolerance X] "
         "<file.x_b|file.bin>\n"
         "Body binding requires a complete transmit/partition stream. "
         "Tolerances are absolute source units; intersection settings are "
         "optional.\n"
         "Schema limits require --nodes, --geometry or --schema-catalog.\n"
         "Environment: OPENPARASOLID_<OPTION>, uppercase with underscores.\n"
         "CLI values override environment; boolean flags accept =false.\n";
}

}  // namespace

int main(int argc, char *argv[]) {
  std::vector<std::string_view> arguments;
  for (int i = 1; i < argc; ++i) arguments.emplace_back(argv[i]);
  const auto configured = openparasolid::cli::ParseInspectSettings(
      arguments, [](std::string_view name) -> std::optional<std::string> {
        const auto key = std::string(name);
        const char *value = std::getenv(key.c_str());
        return value ? std::optional<std::string>(value) : std::nullopt;
      });
  if (!configured.ok()) {
    std::cerr << "Error: " << configured.error().message << '\n';
    Usage(std::cerr);
    return 1;
  }
  const auto &settings = configured.value();
  if (settings.action == openparasolid::cli::InspectAction::kHelp) {
    Usage(std::cout);
    return 0;
  }
  if (settings.action == openparasolid::cli::InspectAction::kVersion) {
    std::cout << OPENPARASOLID_VERSION_STRING << '\n';
    return 0;
  }
  const auto &options = settings.inspect;
  const auto &limits = settings.limits;
  const auto &input = settings.input;
  const auto &catalog_path = settings.catalog;
  const bool summary = settings.summary;
  const bool nodes = settings.nodes;
  if (nodes) {
    try {
      std::optional<openparasolid::SchemaCatalog> catalog;
      if (catalog_path)
        catalog = openparasolid::cli::LoadCatalog(*catalog_path);
      const auto parsed = openparasolid::SchemaParser::ParseFile(
          *input, catalog ? &*catalog : nullptr, limits);
      if (!parsed.ok()) {
        const auto &error = parsed.error();
        std::cerr << "Error: " << openparasolid::ToString(error.code)
                  << " at byte " << error.offset << ": " << error.message
                  << '\n';
        return 1;
      }
      auto report = openparasolid::cli::SchemaReport(
          parsed.value(), *input, summary || settings.geometry);
      report["limits"] = {{"max_input_bytes", limits.max_input_bytes},
                          {"max_nodes", limits.max_nodes},
                          {"max_types", limits.max_types},
                          {"max_fields_per_type", limits.max_fields_per_type},
                          {"max_total_fields", limits.max_total_fields},
                          {"max_values", limits.max_values},
                          {"max_delta_packets", limits.max_delta_packets},
                          {"max_delta_entries", limits.max_delta_entries}};
      int status = parsed.value().complete ? 0 : 2;
      if (settings.geometry) {
        report["interpretation"] = "geometry_model";
        if (settings.body_geometry)
          report["body_geometry_overrides"] =
              openparasolid::cli::BodyOverridesReport(
                  settings.fin_intervals, settings.wire_intervals, summary);
        openparasolid::ModelOptions model_options;
        model_options.max_nodes = limits.max_nodes;
        model_options.max_fields = limits.max_total_fields;
        model_options.max_references = limits.max_values;
        model_options.max_values = limits.max_values;
        const auto model =
            openparasolid::ModelBuilder::Build(parsed.value(), model_options);
        if (!model.ok()) {
          report["model"] = nullptr;
          report["model_error"] = {
              {"code", openparasolid::ToString(model.error().code)},
              {"offset", model.error().offset},
              {"message", model.error().message}};
          status = 2;
          if (settings.body_geometry) {
            report["body_geometry"] = nullptr;
            report["body_geometry_error"] = report["model_error"];
            report["body_geometry_error"]["body"] = *settings.body_geometry;
          }
        } else {
          report["model"] = openparasolid::cli::ModelReport(
              model.value(), model_options, summary);
          if (!model.value().references_verified() ||
              !model.value().typed_geometry_complete())
            status = 2;
          if (settings.body_geometry) {
            report["body_geometry"] = nullptr;
            openparasolid::BodyGeometryBindingOptions binding_options;
            binding_options.intersection = settings.intersection;
            const auto bound = [&]()
                -> openparasolid::Result<openparasolid::BodyGeometryBinding> {
              if (!parsed.value().complete ||
                  (parsed.value().header.kind !=
                       openparasolid::StreamKind::kTransmit &&
                   parsed.value().header.kind !=
                       openparasolid::StreamKind::kPartition))
                return openparasolid::
                    Result<openparasolid::BodyGeometryBinding>::Err(
                        {openparasolid::ErrorCode::kUnsupportedFormat, 0,
                         "Body binding requires a complete transmit or "
                         "partition stream"});
              return openparasolid::BindBodyGeometryWithOverrides(
                  model.value(), openparasolid::NodeId{*settings.body_geometry},
                  settings.fin_intervals, settings.wire_intervals,
                  *settings.binding_tolerance, binding_options);
            }();
            if (!bound.ok()) {
              report["body_geometry_error"] = {
                  {"body", *settings.body_geometry},
                  {"code", openparasolid::ToString(bound.error().code)},
                  {"offset", bound.error().offset},
                  {"message", bound.error().message}};
              status = 2;
            } else {
              report["body_geometry"] = openparasolid::cli::BodyGeometryReport(
                  bound.value(), *settings.binding_tolerance, binding_options,
                  summary);
            }
          }
        }
      }
      std::cout << report.dump(2) << '\n';
      return status;
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
