// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

// swx_dump.cc — Linux CLI tool that demonstrates the openswx library.
//
// Usage: swx_dump [--list-streams] [--dump-streams DIR] [--dump-unknown DIR]
//                 [--extract-parasolid DIR]
//                 [--export-tables-svg FILE [--sheet NAME]]
//                 [--export-view-svg FILE [--view NAME]]
//                 [--export-sheet-svg FILE [--sheet NAME]]
//                 [--export-sheet-preview-png FILE [--sheet NAME]]
//                 [--probe-drawing-geometry]
//                 <file.SLDPRT|file.SLDASM|file.SLDDRW>
//
// Prints a JSON representation of the parsed document to stdout.
// The JSON schema is intentionally compatible with the Windows sw_dumper
// output so that downstream consumers can be shared.

#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

#include "openswx/container.h"
#include "openswx/document.h"
#include "openswx/drawing_geometry.h"
#include "openswx/drawing_probe.h"
#include "openswx/saved_geometry.h"
#include "openswx/svg_writer.h"

namespace {

using nlohmann::json;
using openswx::Configuration;
using openswx::Document;
using openswx::DocumentType;
using openswx::MassProperties;
using openswx::Sheet;

json SerializeMassProperties(const MassProperties& mp) {
  return json{
      {"center_of_gravity",
       {{"x", mp.center_of_gravity[0]},
        {"y", mp.center_of_gravity[1]},
        {"z", mp.center_of_gravity[2]}}},
      {"volume", mp.volume},
      {"surface_area", mp.surface_area},
      {"mass", mp.mass},
      {"moments_of_inertia",
       {mp.moments_of_inertia[0], mp.moments_of_inertia[1],
        mp.moments_of_inertia[2]}},
      {"products_of_inertia",
       {mp.products_of_inertia[0], mp.products_of_inertia[1],
        mp.products_of_inertia[2]}},
  };
}

std::string Base64Encode(const std::vector<uint8_t>& png) {
  static constexpr char kB64Chars[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string b64;
  b64.reserve(((png.size() + 2) / 3) * 4);
  for (std::size_t i = 0; i < png.size(); i += 3) {
    uint32_t grp = static_cast<uint32_t>(png[i]) << 16;
    if (i + 1 < png.size()) grp |= static_cast<uint32_t>(png[i + 1]) << 8;
    if (i + 2 < png.size()) grp |= static_cast<uint32_t>(png[i + 2]);
    b64 += kB64Chars[(grp >> 18) & 0x3F];
    b64 += kB64Chars[(grp >> 12) & 0x3F];
    b64 += (i + 1 < png.size()) ? kB64Chars[(grp >> 6) & 0x3F] : '=';
    b64 += (i + 2 < png.size()) ? kB64Chars[grp & 0x3F] : '=';
  }
  return b64;
}

json SerializeConfiguration(const Configuration& cfg) {
  json jcfg;
  jcfg["name"]  = cfg.name;
  jcfg["index"] = cfg.index;
  if (cfg.derived_from_index)
    jcfg["derived_from_index"] = *cfg.derived_from_index;

  // Properties.
  jcfg["properties"] = json::object();
  for (const auto& [k, v] : cfg.properties) jcfg["properties"][k] = v;

  // Mass properties.
  if (cfg.mass_properties)
    jcfg["mass_properties"] = SerializeMassProperties(*cfg.mass_properties);

  // Preview PNG as base64.
  if (!cfg.preview_png.empty())
    jcfg["preview_png_base64"] = Base64Encode(cfg.preview_png);

  // Cut list.
  json jcl = json::array();
  for (const auto& item : cfg.cut_list) {
    json ji;
    ji["name"]     = item.name;
    ji["quantity"] = item.quantity;
    ji["properties"] = json::object();
    for (const auto& [k, v] : item.properties) ji["properties"][k] = v;
    jcl.push_back(std::move(ji));
  }
  jcfg["cut_list"] = std::move(jcl);

  // Components (assemblies).
  json jcomps = json::array();
  for (const auto& c : cfg.components) {
    jcomps.push_back({
        {"name", c.name},
        {"path", c.path},
        {"ref_config", c.configuration_name},
        {"component_reference", c.component_reference},
        {"is_suppressed", c.is_suppressed},
        {"is_hidden", c.is_hidden},
        {"exclude_from_bom", c.exclude_from_bom},
        {"transform", json(std::vector<double>(c.transform.begin(),
                                               c.transform.end()))},
    });
  }
  jcfg["components"] = std::move(jcomps);

  return jcfg;
}

json SerializeDocument(const Document& doc) {
  json j;

  switch (doc.type) {
    case DocumentType::kPart:     j["doc_type"] = "part";     break;
    case DocumentType::kAssembly: j["doc_type"] = "assembly"; break;
    case DocumentType::kDrawing:  j["doc_type"] = "drawing";  break;
  }

  j["version"] = doc.version;

  j["global_properties"] = json::object();
  for (const auto& [k, v] : doc.global_properties)
    j["global_properties"][k] = v;

  j["configurations"] = json::array();
  for (const auto& cfg : doc.configurations)
    j["configurations"].push_back(SerializeConfiguration(cfg));

  j["sheets"] = json::array();
  for (const auto& sheet : doc.sheets) {
    json js;
    js["name"] = sheet.name;
    // Drawings (.SLDDRW) have no configurations, so their preview lives
    // only on the sheet — see Sheet::preview_png in types.h, sourced from
    // the Images/Sheet_N streams in document.cc.
    if (!sheet.preview_png.empty())
      js["preview_png_base64"] = Base64Encode(sheet.preview_png);
    js["views"] = json::array();
    for (const auto& v : sheet.views) {
      js["views"].push_back({
          {"name", v.name},
          {"referenced_document", v.referenced_document},
          {"referenced_config", v.referenced_configuration},
      });
    }
    j["sheets"].push_back(std::move(js));
  }

  j["replacement_references"] = doc.replacement_references;
  j["header_references"] = doc.header_references;

  return j;
}

std::string HexEncode(std::span<const uint8_t> value) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string encoded;
  encoded.reserve(value.size() * 2);
  for (const uint8_t byte : value) {
    encoded.push_back(kHex[byte >> 4]);
    encoded.push_back(kHex[byte & 0x0Fu]);
  }
  return encoded;
}

std::string HexEncode(std::string_view value) {
  return HexEncode(std::span<const uint8_t>(
      reinterpret_cast<const uint8_t*>(value.data()), value.size()));
}

json SerializeInspection(const openswx::ContainerInspection& inspection) {
  json result;
  result["container_kind"] = "modern";
  result["coverage"] = {
      {"file_size", inspection.file_size},
      {"claimed_bytes", inspection.claimed_bytes},
      {"unknown_bytes", inspection.unknown_bytes},
  };
  result["streams"] = json::array();
  for (const auto& stream : inspection.streams) {
    json item = {
        {"marker_offset", stream.marker_offset},
        {"type_id", stream.type_id},
        {"compressed_size", stream.compressed_size},
        {"uncompressed_size", stream.uncompressed_size},
        {"decoded_size", stream.decoded_size},
        {"expected_crc32", stream.expected_crc32},
        {"status", openswx::ToString(stream.status)},
        {"size_matches", stream.size_matches},
        {"crc_matches", stream.crc_matches},
        {"duplicate_name", stream.duplicate_name},
        {"selected_as_primary", stream.selected_as_primary},
    };
    item["record_size"] = stream.record_size;
    item["name"] = stream.name;
    if (stream.name.starts_with("SectionBodies/") &&
        stream.status == openswx::StreamDecodeStatus::kDecoded) {
      item["embedded_parasolid_streams"] = json::array();
      for (const auto& embedded :
           openswx::ExtractEmbeddedParasolidStreams(stream.decoded_data)) {
        item["embedded_parasolid_streams"].push_back({
            {"marker_offset", embedded.marker_offset},
            {"compressed_size", embedded.compressed_size},
            {"uncompressed_size", embedded.uncompressed_size},
        });
      }
    }
    result["streams"].push_back(std::move(item));
  }

  result["cache_cells"] = json::array();
  for (const auto& cell : inspection.cache_cells) {
    result["cache_cells"].push_back({
        {"marker_offset", cell.marker_offset},
        {"record_size", cell.record_size},
        {"type_id", cell.type_id},
        {"logical_size", cell.logical_size},
        {"name", cell.name},
    });
  }

  result["directory_entries"] = json::array();
  for (const auto& entry : inspection.directory_entries) {
    result["directory_entries"].push_back({
        {"marker_offset", entry.marker_offset},
        {"record_size", entry.record_size},
        {"type_id", entry.type_id},
        {"stored_size", entry.stored_size},
        {"name", entry.name},
        {"descriptor_hex", HexEncode(entry.descriptor)},
        {"trailer_hex", HexEncode(entry.trailer)},
    });
  }

  result["unknown_markers"] = json::array();
  for (const auto& marker : inspection.unknown_markers) {
    json item = {
        {"marker_offset", marker.marker_offset},
        {"type_id", marker.type_id},
        {"inline_decode_status",
         openswx::ToString(marker.inline_decode_status)},
    };
    const bool printable = !marker.candidate_name.empty() &&
        std::all_of(marker.candidate_name.begin(), marker.candidate_name.end(),
                    [](unsigned char c) { return c >= 0x20 && c < 0x7F; });
    if (printable) {
      item["candidate_name"] = marker.candidate_name;
    } else {
      item["candidate_name"] = nullptr;
      item["candidate_name_hex"] = HexEncode(marker.candidate_name);
    }
    result["unknown_markers"].push_back(std::move(item));
  }

  result["unknown_regions"] = json::array();
  for (const auto& region : inspection.unknown_regions) {
    result["unknown_regions"].push_back({
        {"offset", region.offset},
        {"size", region.size},
    });
  }
  return result;
}

std::optional<std::filesystem::path> SafeRelativeStreamPath(
    std::string name) {
  std::replace(name.begin(), name.end(), '\\', '/');
  const std::filesystem::path path(name);
  if (path.empty() || path.is_absolute()) return std::nullopt;
  for (const auto& component : path) {
    if (component == "." || component == "..") return std::nullopt;
  }
  return path;
}

bool DumpStreams(const openswx::ContainerInspection& inspection,
                 const std::filesystem::path& output_dir) {
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  if (error) {
    std::cerr << "Cannot create output directory: " << error.message() << "\n";
    return false;
  }

  std::unordered_map<std::string, std::size_t> name_counts;
  for (const auto& stream : inspection.streams) {
    if (!stream.name.empty()) ++name_counts[stream.name];
  }

  std::unordered_map<std::string, std::size_t> duplicate_indices;
  for (const auto& stream : inspection.streams) {
    if (stream.status != openswx::StreamDecodeStatus::kDecoded &&
        stream.status != openswx::StreamDecodeStatus::kEmpty) {
      continue;
    }

    auto relative = SafeRelativeStreamPath(stream.name);
    if (!relative) {
      std::cerr << "Refusing unsafe stream path: " << stream.name << "\n";
      return false;
    }
    if (name_counts[stream.name] > 1) {
      const std::size_t index = duplicate_indices[stream.name]++;
      *relative += ".chunk-" + std::to_string(index);
    }

    const std::filesystem::path output_path = output_dir / *relative;
    std::filesystem::create_directories(output_path.parent_path(), error);
    if (error) {
      std::cerr << "Cannot create stream directory: " << error.message()
                << "\n";
      return false;
    }

    std::ofstream output(output_path, std::ios::binary);
    if (!output.is_open()) {
      std::cerr << "Cannot write stream: " << output_path << "\n";
      return false;
    }
    output.write(reinterpret_cast<const char*>(stream.decoded_data.data()),
                 static_cast<std::streamsize>(stream.decoded_data.size()));
    if (!output) {
      std::cerr << "Write failed for stream: " << output_path << "\n";
      return false;
    }
  }
  return true;
}

bool DumpUnknownRegions(const std::filesystem::path& input_path,
                        const openswx::ContainerInspection& inspection,
                        const std::filesystem::path& output_dir) {
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  if (error) {
    std::cerr << "Cannot create unknown-region directory: "
              << error.message() << "\n";
    return false;
  }

  std::ifstream input(input_path, std::ios::binary);
  if (!input.is_open()) {
    std::cerr << "Cannot reopen input file: " << input_path << "\n";
    return false;
  }

  std::array<char, 64 * 1024> buffer{};
  for (const auto& region : inspection.unknown_regions) {
    const std::string filename =
        "unknown-" + std::to_string(region.offset) + "-" +
        std::to_string(region.size) + ".bin";
    std::ofstream output(output_dir / filename, std::ios::binary);
    if (!output.is_open()) {
      std::cerr << "Cannot write unknown region: " << filename << "\n";
      return false;
    }

    input.clear();
    input.seekg(static_cast<std::streamoff>(region.offset));
    uint64_t remaining = region.size;
    while (remaining > 0) {
      const std::size_t chunk = static_cast<std::size_t>(
          std::min<uint64_t>(remaining, buffer.size()));
      input.read(buffer.data(), static_cast<std::streamsize>(chunk));
      if (input.gcount() != static_cast<std::streamsize>(chunk)) {
        std::cerr << "Read failed for unknown region at " << region.offset
                  << "\n";
        return false;
      }
      output.write(buffer.data(), static_cast<std::streamsize>(chunk));
      if (!output) {
        std::cerr << "Write failed for unknown region: " << filename << "\n";
        return false;
      }
      remaining -= chunk;
    }
  }

  const std::filesystem::path marker_dir = output_dir / "markers";
  std::filesystem::create_directories(marker_dir, error);
  if (error) {
    std::cerr << "Cannot create unknown-marker directory: "
              << error.message() << "\n";
    return false;
  }
  for (const auto& marker : inspection.unknown_markers) {
    const std::string filename =
        "unknown-marker-" + std::to_string(marker.marker_offset) + ".bin";
    std::ofstream output(marker_dir / filename, std::ios::binary);
    if (!output.is_open()) {
      std::cerr << "Cannot write unknown marker: " << filename << "\n";
      return false;
    }
    input.clear();
    input.seekg(static_cast<std::streamoff>(marker.marker_offset));
    std::array<char, 6> marker_bytes{};
    input.read(marker_bytes.data(),
               static_cast<std::streamsize>(marker_bytes.size()));
    if (input.gcount() !=
        static_cast<std::streamsize>(marker_bytes.size())) {
      std::cerr << "Read failed for unknown marker at "
                << marker.marker_offset << "\n";
      return false;
    }
    output.write(marker_bytes.data(),
                 static_cast<std::streamsize>(marker_bytes.size()));
    if (!output) {
      std::cerr << "Write failed for unknown marker: " << filename << "\n";
      return false;
    }
  }
  return true;
}

const openswx::StreamInfo* FindDecodedStream(
    const openswx::ContainerInspection& inspection, std::string_view name) {
  const openswx::StreamInfo* fallback = nullptr;
  for (const auto& stream : inspection.streams) {
    if (stream.name != name ||
        (stream.status != openswx::StreamDecodeStatus::kDecoded &&
         stream.status != openswx::StreamDecodeStatus::kEmpty)) {
      continue;
    }
    if (stream.selected_as_primary) return &stream;
    if (fallback == nullptr) fallback = &stream;
  }
  return fallback;
}

const openswx::StreamInfo* FindDecodedStreamPrefix(
    const openswx::ContainerInspection& inspection, std::string_view prefix) {
  for (const auto& stream : inspection.streams) {
    if (stream.name.starts_with(prefix) &&
        stream.status == openswx::StreamDecodeStatus::kDecoded) {
      return &stream;
    }
  }
  return nullptr;
}

bool WriteTextFile(const std::filesystem::path& path,
                   const std::string& contents) {
  std::ofstream output(path, std::ios::binary);
  if (!output.is_open()) {
    std::cerr << "Cannot open output file: " << path << "\n";
    return false;
  }
  output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  if (!output) {
    std::cerr << "Cannot write output file: " << path << "\n";
    return false;
  }
  return true;
}

bool WriteBinaryFile(const std::filesystem::path& path,
                     std::span<const uint8_t> contents) {
  std::ofstream output(path, std::ios::binary);
  if (!output.is_open()) {
    std::cerr << "Cannot open output file: " << path << "\n";
    return false;
  }
  output.write(reinterpret_cast<const char*>(contents.data()),
               static_cast<std::streamsize>(contents.size()));
  if (!output) {
    std::cerr << "Cannot write output file: " << path << "\n";
    return false;
  }
  return true;
}

std::string ParasolidCategory(const openswx::ParasolidPayload &payload) {
  if (payload.source_stream.starts_with("SectionBodies/"))
    return "section-bodies";
  if (payload.ghost_partition)
    return "ghost-partitions";
  if (payload.kind == openswx::ParasolidPayloadKind::kPartition)
    return "partitions";
  if (payload.kind == openswx::ParasolidPayloadKind::kDeltas)
    return "deltas";
  return "embedded";
}

bool ExportParasolid(const openswx::ContainerInspection &inspection,
                     const std::filesystem::path &input,
                     const std::filesystem::path &directory) {
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    std::cerr << "Cannot create Parasolid output directory: " << error.message()
              << "\n";
    return false;
  }
  const bool empty = std::filesystem::is_empty(directory, error);
  if (error || !empty) {
    std::cerr << "Parasolid output directory must be empty: " << directory
              << "\n";
    return false;
  }
  const auto extracted = openswx::ExtractParasolidStreams(inspection);
  std::map<uint32_t, std::string> configurations;
  auto document = openswx::SwxDocument::Open(input);
  if (document.ok()) {
    for (const auto &configuration : document.value().configurations())
      configurations.emplace(configuration.index, configuration.name);
  }
  json manifest = {
      {"schema_version", 1},
      {"source_file", input.string()},
      {"source_file_size", inspection.file_size},
      {"format", "binary_parasolid"},
      {"scope", "primary_decoded_streams"},
      {"inspected_stream_count", extracted.inspected_stream_count},
      {"wrapper_count", extracted.wrapper_count},
      {"non_parasolid_block_count", extracted.non_parasolid_block_count},
      {"payload_count", extracted.payloads.size()},
      {"issue_count", extracted.issues.size()},
      {"verification", "framing_sizes_zlib_checksums_and_signature"},
      {"standalone_readability", "unverified"},
      {"payloads", json::array()},
      {"issues", json::array()}};
  for (std::size_t index = 0; index < extracted.payloads.size(); ++index) {
    const auto &payload = extracted.payloads[index];
    const auto &block = payload.payload;
    const std::string category = ParasolidCategory(payload);
    const std::string extension =
        payload.header_valid &&
                payload.kind != openswx::ParasolidPayloadKind::kDeltas &&
                payload.kind != openswx::ParasolidPayloadKind::kUnknown
            ? ".x_b"
            : ".bin";
    const auto relative = std::filesystem::path(category) /
                          ("payload-" + std::to_string(index) + "-offset-" +
                           std::to_string(block.marker_offset) + extension);
    std::filesystem::create_directories(directory / category, error);
    if (error) {
      std::cerr << "Cannot create Parasolid category directory: "
                << error.message() << "\n";
      return false;
    }
    if (!WriteBinaryFile(directory / relative, block.data))
      return false;
    json entry = {
        {"index", index},
        {"file", relative.generic_string()},
        {"category", category},
        {"kind", openswx::ToString(payload.kind)},
        {"source_stream", payload.source_stream},
        {"container_marker_offset", payload.container_marker_offset},
        {"marker_offset", block.marker_offset},
        {"compressed_size", block.compressed_size},
        {"uncompressed_size", block.uncompressed_size},
        {"bytes", block.data.size()},
        {"frame_count", block.frames.size()},
        {"section_offset",
         block.section_offset ? json(*block.section_offset) : json(nullptr)},
        {"stored_size", block.stored_size},
        {"frames", json::array()},
        {"header_valid", payload.header_valid},
        {"description", payload.description},
        {"schema", payload.schema},
        {"ghost_partition", payload.ghost_partition},
        {"configuration_index", payload.configuration_index
                                    ? json(*payload.configuration_index)
                                    : json(nullptr)},
        {"configuration_name", nullptr}};
    for (const auto &frame : block.frames) {
      entry["frames"].push_back(
          {{"header_offset", frame.header_offset},
           {"compressed_size", frame.compressed_size},
           {"uncompressed_size", frame.uncompressed_size}});
    }
    if (payload.configuration_index) {
      auto config = configurations.find(*payload.configuration_index);
      if (config != configurations.end())
        entry["configuration_name"] = config->second;
    }
    if (payload.kind == openswx::ParasolidPayloadKind::kDeltas) {
      entry["requires_base"] = true;
      entry["dependency_resolution"] = "not_attempted";
      entry["base_partition_candidate"] = nullptr;
      // A preceding partition in the same stream is a useful candidate, not
      // proof of a kernel-level dependency or a complete reconstruction.
      for (std::size_t previous = index; previous > 0u; --previous) {
        const auto &candidate = extracted.payloads[previous - 1u];
        if (candidate.source_stream == payload.source_stream &&
            candidate.kind == openswx::ParasolidPayloadKind::kPartition) {
          entry["base_partition_candidate"] = previous - 1u;
          break;
        }
      }
    }
    manifest["payloads"].push_back(std::move(entry));
  }
  for (const auto &issue : extracted.issues) {
    manifest["issues"].push_back(
        {{"source_stream", issue.source_stream},
         {"marker_offset",
          issue.marker_offset ? json(*issue.marker_offset) : json(nullptr)},
         {"message", issue.message}});
  }
  if (!WriteTextFile(directory / "manifest.json", manifest.dump(2) + "\n"))
    return false;
  std::cout << manifest.dump(2) << "\n";
  // Retain recoverable payloads and the report, but make incomplete extraction
  // visible to scripts as well as interactive users.
  return extracted.issues.empty();
}

json SerializeDrawingProbe(const openswx::DrawingBinaryProbe& probe) {
  std::map<std::string, std::size_t> class_counts;
  for (const auto& declaration : probe.classes) {
    ++class_counts[declaration.name];
  }
  json classes = json::object();
  for (const auto& [name, count] : class_counts) classes[name] = count;
  return json{
      {"byte_size", probe.byte_size},
      {"class_declaration_count", probe.classes.size()},
      {"class_counts", std::move(classes)},
      {"sketch_markers",
       {{"current", probe.current_sketch_markers},
        {"legacy", probe.legacy_sketch_markers},
        {"legacy_extended", probe.legacy_extended_sketch_markers}}},
      {"coordinate_pair_candidates", probe.coordinate_pair_candidates},
      {"coordinate_bounds_m",
       probe.has_coordinate_bounds
           ? json{{"min_x", probe.coordinate_min_x},
                  {"min_y", probe.coordinate_min_y},
                  {"max_x", probe.coordinate_max_x},
                  {"max_y", probe.coordinate_max_y}}
           : json(nullptr)},
  };
}

json ProbeDrawingStreams(const openswx::ContainerInspection& inspection) {
  static constexpr std::array<std::string_view, 3> kStreams = {
      "Contents/Definition", "Contents/DisplayLists", "Contents/VBLists"};
  json streams = json::object();
  for (const std::string_view name : kStreams) {
    const openswx::StreamInfo* selected = nullptr;
    for (const auto& stream : inspection.streams) {
      if (stream.name == name &&
          stream.status == openswx::StreamDecodeStatus::kDecoded &&
          (selected == nullptr || stream.selected_as_primary)) {
        selected = &stream;
      }
    }
    if (selected == nullptr) {
      streams[std::string(name)] = nullptr;
    } else {
      json stream_probe = SerializeDrawingProbe(
          openswx::ProbeDrawingBinary(selected->decoded_data));
      if (name == "Contents/VBLists") {
        const auto appearances =
            openswx::DrawingGeometryReader::ReadAppearances(
                selected->decoded_data);
        json appearance_records = json::array();
        for (const auto& appearance : appearances) {
          appearance_records.push_back(
              {{"name", appearance.name},
               {"rgb", appearance.rgb},
               {"opacity", appearance.opacity},
               {"source_offset", appearance.source_offset},
               {"scope",
                appearance.scope ==
                        openswx::DrawingAppearanceScope::kClassDefinition
                    ? "class_definition"
                    : "inline_unresolved"}});
        }
        stream_probe["appearances"] =
            {{"count", appearances.size()},
             {"records", std::move(appearance_records)}};
        const auto lights = openswx::DrawingGeometryReader::ReadSceneLights(
            selected->decoded_data);
        json light_records = json::array();
        for (const auto& light : lights) {
          light_records.push_back(
              {{"kind",
                light.kind == openswx::DrawingLightKind::kAmbient
                    ? "ambient"
                    : "directional"},
               {"name", light.name},
               {"ambient", light.ambient},
               {"diffuse", light.diffuse},
               {"specular", light.specular},
               {"rgb", light.rgb},
               {"enabled", light.enabled},
               {"direction",
                {light.direction.x, light.direction.y, light.direction.z}},
               {"source_offset", light.source_offset}});
        }
        stream_probe["scene_lights"] =
            {{"count", lights.size()}, {"records", std::move(light_records)}};
        auto tessellations =
            openswx::DrawingGeometryReader::ReadFaceTessellations(
                selected->decoded_data);
        if (!tessellations.ok()) {
          stream_probe["face_tessellation"] =
              json{{"decoded", false}, {"error", tessellations.error()}};
        } else {
          std::size_t vertices = 0u;
          std::size_t triangles = 0u;
          std::size_t strips = 0u;
          for (const auto& mesh : tessellations.value()) {
            vertices += mesh.vertices.size();
            triangles += mesh.triangles.size();
            strips += mesh.strip_lengths.size();
          }
          stream_probe["face_tessellation"] =
              json{{"decoded", true},
                   {"face_count", tessellations.value().size()},
                   {"vertex_count", vertices},
                   {"triangle_count", triangles},
                   {"strip_count", strips}};
        }
      }
      streams[std::string(name)] = std::move(stream_probe);
    }
  }
  return streams;
}

json SavedGeometryJson(const openswx::SavedGeometryResolution& result) {
  const auto source_json = [](const openswx::GeometrySource& source) {
    return json{{"stream", source.stream},
                {"container_offset", source.container_offset},
                {"payload_offset", source.payload_offset}};
  };
  json report{
      {"schema_version", 1},
      {"configuration", result.configuration},
      {"configuration_status", openswx::ToString(result.configuration_status)},
      {"partition_status", openswx::ToString(result.partition_status)},
      {"state_status", openswx::ToString(result.state_status)},
      {"body_membership_status",
       openswx::ToString(result.body_membership_status)},
      {"partition_index",
       result.partition_index ? json(*result.partition_index) : json(nullptr)},
      {"resolved_features_index", result.resolved_features_index
                                      ? json(*result.resolved_features_index)
                                      : json(nullptr)},
      {"was_last_active",
       result.was_last_active ? json(*result.was_last_active) : json(nullptr)},
      {"needs_rebuild",
       result.needs_rebuild ? json(*result.needs_rebuild) : json(nullptr)},
      {"limitations", result.limitations},
      {"evidence", json::array()},
      {"candidates", json::array()}};
  for (const auto& evidence : result.evidence)
    report["evidence"].push_back({{"source", source_json(evidence.source)},
                                  {"field", evidence.field},
                                  {"value", evidence.value}});
  for (const auto& candidate : result.candidates) {
    json row{
        {"payload_index", candidate.payload_index},
        {"source", source_json(candidate.source)},
        {"role", openswx::ToString(candidate.role)},
        {"kind", openswx::ToString(candidate.kind)},
        {"stream_index", candidate.stream_index ? json(*candidate.stream_index)
                                                : json(nullptr)},
        {"selected_partition", candidate.selected_partition},
        {"decoded_complete", candidate.decoded_complete},
        {"decode_issue", candidate.decode_issue},
        {"body_candidates", json::array()}};
    for (const auto& body : candidate.bodies)
      row["body_candidates"].push_back(
          {{"body_id", body.body_id}, {"kind", openswx::ToString(body.kind)}});
    report["candidates"].push_back(std::move(row));
  }
  return report;
}

struct Options {
  std::filesystem::path input;
  bool list_streams = false;
  std::optional<std::string> resolve_saved_geometry;
  std::optional<std::filesystem::path> extract_parasolid;
  std::optional<std::filesystem::path> dump_streams;
  std::optional<std::filesystem::path> dump_unknown;
  std::optional<std::filesystem::path> export_tables_svg;
  std::optional<std::filesystem::path> export_view_svg;
  std::optional<std::filesystem::path> export_sheet_svg;
  std::optional<std::filesystem::path> export_sheet_preview_png;
  std::optional<std::string> sheet;
  std::optional<std::string> view;
  bool probe_drawing_geometry = false;
};

std::optional<Options> ParseOptions(int argc, char* argv[]) {
  Options options;
  if (const char* value = std::getenv("SWX_DUMP_RESOLVE_SAVED_GEOMETRY");
      value && *value)
    options.resolve_saved_geometry = value;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--list-streams") {
      options.list_streams = true;
    } else if (argument == "--resolve-saved-geometry") {
      if (++i >= argc || std::string_view(argv[i]).empty()) return std::nullopt;
      options.resolve_saved_geometry = argv[i];
    } else if (argument == "--extract-parasolid") {
      if (++i >= argc)
        return std::nullopt;
      options.extract_parasolid = std::filesystem::path(argv[i]);
    } else if (argument == "--dump-streams") {
      if (++i >= argc) return std::nullopt;
      options.dump_streams = std::filesystem::path(argv[i]);
    } else if (argument == "--dump-unknown") {
      if (++i >= argc) return std::nullopt;
      options.dump_unknown = std::filesystem::path(argv[i]);
    } else if (argument == "--export-tables-svg") {
      if (++i >= argc) return std::nullopt;
      options.export_tables_svg = std::filesystem::path(argv[i]);
    } else if (argument == "--export-sheet-preview-png") {
      if (++i >= argc) return std::nullopt;
      options.export_sheet_preview_png = std::filesystem::path(argv[i]);
    } else if (argument == "--export-view-svg") {
      if (++i >= argc) return std::nullopt;
      options.export_view_svg = std::filesystem::path(argv[i]);
    } else if (argument == "--export-sheet-svg") {
      if (++i >= argc) return std::nullopt;
      options.export_sheet_svg = std::filesystem::path(argv[i]);
    } else if (argument == "--probe-drawing-geometry") {
      options.probe_drawing_geometry = true;
    } else if (argument == "--sheet") {
      if (++i >= argc) return std::nullopt;
      options.sheet = argv[i];
    } else if (argument == "--view") {
      if (++i >= argc) return std::nullopt;
      options.view = argv[i];
    } else if (!argument.empty() && argument[0] == '-') {
      return std::nullopt;
    } else if (options.input.empty()) {
      options.input = argument;
    } else {
      return std::nullopt;
    }
  }
  if (options.input.empty()) return std::nullopt;
  const int exclusive_actions = (options.resolve_saved_geometry ? 1 : 0) +
                                (options.extract_parasolid ? 1 : 0) +
                                (options.export_tables_svg ? 1 : 0) +
                                (options.export_view_svg ? 1 : 0) +
                                (options.export_sheet_svg ? 1 : 0) +
                                (options.export_sheet_preview_png ? 1 : 0) +
                                (options.probe_drawing_geometry ? 1 : 0);
  if (exclusive_actions > 1) return std::nullopt;
  if (options.sheet && !options.export_tables_svg &&
      !options.export_sheet_preview_png && !options.export_sheet_svg) {
    return std::nullopt;
  }
  if (options.view && !options.export_view_svg) return std::nullopt;
  if (exclusive_actions != 0 &&
      (options.list_streams || options.dump_streams || options.dump_unknown)) {
    return std::nullopt;
  }
  return options;
}

}  // namespace

int main(int argc, char* argv[]) {
  auto options = ParseOptions(argc, argv);
  if (!options) {
    std::cerr << "Usage: swx_dump [--list-streams] [--dump-streams DIR] "
                 "[--dump-unknown DIR] [--extract-parasolid DIR] "
                 "[--resolve-saved-geometry CONFIG] "
                 "[--export-tables-svg FILE "
                 "[--sheet NAME]] [--export-view-svg FILE [--view NAME]] "
                 "[--export-sheet-svg FILE [--sheet NAME]] "
                 "[--export-sheet-preview-png FILE "
                 "[--sheet NAME]] [--probe-drawing-geometry] "
                 "<SolidWorks file>\n";
    return 1;
  }

  if (options->resolve_saved_geometry) {
    const auto result = openswx::ResolveSavedGeometry(
        options->input, *options->resolve_saved_geometry);
    if (!result.ok()) {
      std::cerr << "Error: " << result.error() << "\n";
      return 1;
    }
    auto report = SavedGeometryJson(result.value());
    report["source_file"] = options->input.string();
    std::cout << report.dump(2) << "\n";
    return 0;
  }

  if (options->extract_parasolid) {
    auto inspection = openswx::InspectContainer(options->input);
    if (!inspection.ok()) {
      std::cerr << "Error: " << inspection.error() << "\n";
      return 1;
    }
    return ExportParasolid(inspection.value(), options->input,
                           *options->extract_parasolid)
               ? 0
               : 1;
  }

  if (options->export_sheet_preview_png) {
    auto document = openswx::SwxDocument::Open(options->input);
    if (!document.ok()) {
      std::cerr << "Error: " << document.error() << "\n";
      return 1;
    }
    const openswx::Sheet* selected = nullptr;
    for (const auto& sheet : document.value().sheets()) {
      if ((!options->sheet || sheet.name == *options->sheet) &&
          !sheet.preview_png.empty()) {
        selected = &sheet;
        break;
      }
    }
    if (selected == nullptr) {
      std::cerr << "Error: requested sheet preview was not found\n";
      return 1;
    }
    if (!WriteBinaryFile(*options->export_sheet_preview_png,
                         selected->preview_png)) {
      return 1;
    }
    std::cout << json{{"output", options->export_sheet_preview_png->string()},
                      {"sheet", selected->name},
                      {"bytes", selected->preview_png.size()},
                      {"format", "png"}}
                     .dump(2)
              << "\n";
    return 0;
  }

  if (options->probe_drawing_geometry) {
    auto inspection = openswx::InspectContainer(options->input);
    if (!inspection.ok()) {
      std::cerr << "Error: " << inspection.error() << "\n";
      return 1;
    }
    std::cout << json{{"drawing_streams",
                       ProbeDrawingStreams(inspection.value())}}
                     .dump(2)
              << "\n";
    return 0;
  }

  if (options->export_sheet_svg) {
    auto inspection = openswx::InspectContainer(options->input);
    if (!inspection.ok()) {
      std::cerr << "Error: " << inspection.error() << "\n";
      return 1;
    }
    const openswx::StreamInfo* keywords = FindDecodedStream(
        inspection.value(), "swXmlContents/KeyWords");
    const openswx::StreamInfo* definition = FindDecodedStream(
        inspection.value(), "Contents/Definition");
    const openswx::StreamInfo* vb_lists = FindDecodedStream(
        inspection.value(), "Contents/VBLists");
    const openswx::StreamInfo* display_lists = FindDecodedStream(
        inspection.value(), "Contents/DisplayLists");
    const openswx::StreamInfo* tables = FindDecodedStream(
        inspection.value(), "swXmlContents/Tables");
    const openswx::StreamInfo* sketch_bitmap = FindDecodedStreamPrefix(
        inspection.value(), "Contents/SketchBitmap");
    const openswx::StreamInfo* printing = FindDecodedStream(
        inspection.value(), "Contents/Printing");
    if (keywords == nullptr || definition == nullptr || vb_lists == nullptr) {
      std::cerr << "Error: KeyWords, Definition or VBLists stream was not found\n";
      return 1;
    }
    auto sheet = openswx::DrawingGeometryReader::ReadCachedSheet(
        keywords->decoded_data, definition->decoded_data,
        vb_lists->decoded_data,
        tables != nullptr ? std::span<const uint8_t>(tables->decoded_data)
                          : std::span<const uint8_t>{},
        options->sheet.value_or(""),
        display_lists != nullptr
            ? std::span<const uint8_t>(display_lists->decoded_data)
            : std::span<const uint8_t>{},
        sketch_bitmap != nullptr
            ? std::span<const uint8_t>(sketch_bitmap->decoded_data)
            : std::span<const uint8_t>{},
        printing != nullptr ? std::span<const uint8_t>(printing->decoded_data)
                            : std::span<const uint8_t>{});
    if (!sheet.ok()) {
      std::cerr << "Error: " << sheet.error() << "\n";
      return 1;
    }
    auto svg = openswx::SvgWriter::WriteSheet(sheet.value());
    if (!svg.ok()) {
      std::cerr << "Error: " << svg.error() << "\n";
      return 1;
    }
    if (!WriteTextFile(*options->export_sheet_svg, svg.value())) return 1;
    std::size_t lines = 0;
    std::size_t arcs = 0;
    std::size_t texts = 0;
    std::size_t dimensions = 0;
    std::size_t balloons = 0;
    std::size_t filled_triangles = 0;
    std::size_t raster_images = 0;
    for (const auto& entity : sheet.value().entities) {
      if (std::holds_alternative<openswx::Line2D>(entity)) ++lines;
      if (std::holds_alternative<openswx::Arc2D>(entity)) ++arcs;
      if (std::holds_alternative<openswx::FilledTriangle2D>(entity)) {
        ++filled_triangles;
      }
      if (std::holds_alternative<openswx::RasterImage2D>(entity)) {
        ++raster_images;
      }
      if (std::holds_alternative<openswx::TextAnnotation>(entity)) ++texts;
      if (std::holds_alternative<openswx::Dimension>(entity)) ++dimensions;
      if (std::holds_alternative<openswx::BomBalloon>(entity)) ++balloons;
    }
    std::cout << json{{"output", options->export_sheet_svg->string()},
                      {"sheet", sheet.value().name},
                      {"width_m", sheet.value().width_m},
                      {"height_m", sheet.value().height_m},
                       {"line_count", lines},
                       {"arc_count", arcs},
                      {"filled_triangle_count", filled_triangles},
                      {"raster_image_count", raster_images},
                      {"text_count", texts},
                      {"dimension_count", dimensions},
                      {"balloon_count", balloons},
                      {"format", "svg"}}
                     .dump(2)
              << "\n";
    return 0;
  }

  if (options->export_view_svg) {
    auto inspection = openswx::InspectContainer(options->input);
    if (!inspection.ok()) {
      std::cerr << "Error: " << inspection.error() << "\n";
      return 1;
    }
    const openswx::StreamInfo* stream =
        FindDecodedStream(inspection.value(), "Contents/VBLists");
    if (stream == nullptr) {
      std::cerr << "Error: Contents/VBLists stream was not found\n";
      return 1;
    }
    auto views = openswx::DrawingGeometryReader::ReadViewBuckets(
        stream->decoded_data);
    if (!views.ok()) {
      std::cerr << "Error: " << views.error() << "\n";
      return 1;
    }
    const openswx::CachedDrawingView* selected = nullptr;
    for (const auto& view : views.value()) {
      if (!options->view || view.name == *options->view) {
        selected = &view;
        break;
      }
    }
    if (selected == nullptr) {
      std::cerr << "Error: requested cached drawing view was not found\n";
      return 1;
    }
    auto svg = openswx::SvgWriter::WriteSheet(selected->geometry);
    if (!svg.ok()) {
      std::cerr << "Error: " << svg.error() << "\n";
      return 1;
    }
    if (!WriteTextFile(*options->export_view_svg, svg.value())) return 1;
    std::cout << json{{"output", options->export_view_svg->string()},
                      {"view", selected->name},
                      {"referenced_document", selected->referenced_document},
                      {"source_offset", selected->source_offset},
                      {"scale_denominator", selected->scale_denominator},
                      {"line_count", selected->geometry.entities.size()},
                      {"format", "svg"}}
                     .dump(2)
              << "\n";
    return 0;
  }

  if (options->export_tables_svg) {
    auto inspection = openswx::InspectContainer(options->input);
    if (!inspection.ok()) {
      std::cerr << "Error: " << inspection.error() << "\n";
      return 1;
    }
    const openswx::StreamInfo* stream = FindDecodedStream(
        inspection.value(), "swXmlContents/Tables");
    if (stream == nullptr) {
      std::cerr << "Error: swXmlContents/Tables stream was not found\n";
      return 1;
    }
    auto geometry = openswx::DrawingGeometryReader::ReadTablesXml(
        stream->decoded_data);
    if (!geometry.ok()) {
      std::cerr << "Error: " << geometry.error() << "\n";
      return 1;
    }
    const openswx::DrawingSheetGeometry* selected = nullptr;
    for (const auto& sheet : geometry.value().sheets) {
      if (!options->sheet || sheet.name == *options->sheet) {
        selected = &sheet;
        break;
      }
    }
    if (selected == nullptr) {
      std::cerr << "Error: requested sheet was not found in Tables XML\n";
      return 1;
    }
    auto svg = openswx::SvgWriter::WriteSheet(*selected);
    if (!svg.ok()) {
      std::cerr << "Error: " << svg.error() << "\n";
      return 1;
    }
    if (!WriteTextFile(*options->export_tables_svg, svg.value())) return 1;

    std::size_t lines = 0;
    std::size_t texts = 0;
    for (const auto& entity : selected->entities) {
      if (std::holds_alternative<openswx::Line2D>(entity)) ++lines;
      if (std::holds_alternative<openswx::TextAnnotation>(entity)) ++texts;
    }
    std::cout << json{{"output", options->export_tables_svg->string()},
                      {"sheet", selected->name},
                      {"entities", selected->entities.size()},
                      {"line_count", lines},
                      {"text_count", texts}}
                     .dump(2)
              << "\n";
    return 0;
  }

  if (options->list_streams || options->dump_streams || options->dump_unknown) {
    auto inspection = openswx::InspectContainer(options->input);
    if (!inspection.ok()) {
      std::cerr << "Error: " << inspection.error() << "\n";
      return 1;
    }
    if (options->dump_streams &&
        !DumpStreams(inspection.value(), *options->dump_streams)) {
      return 1;
    }
    if (options->dump_unknown &&
        !DumpUnknownRegions(options->input, inspection.value(),
                            *options->dump_unknown)) {
      return 1;
    }
    std::cout << SerializeInspection(inspection.value()).dump(2) << "\n";
    return 0;
  }

  auto result = openswx::SwxDocument::Open(options->input);
  if (!result.ok()) {
    std::cerr << "Error: " << result.error() << "\n";
    return 1;
  }

  std::cout << SerializeDocument(result.value().doc()).dump(2) << "\n";
  return 0;
}
