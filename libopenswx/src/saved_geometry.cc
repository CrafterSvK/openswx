// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "openswx/saved_geometry.h"

#include <algorithm>
#include <charconv>
#include <limits>
#include <map>
#include <pugixml.hpp>
#include <set>
#include <utility>

#include "openparasolid/model.h"
#include "openparasolid/schema.h"
#include "saved_geometry_history.h"

namespace openswx {
namespace {
constexpr std::string_view kSwNamespace =
    "http://www.solidworks.com/sw2003/schema";
std::optional<uint32_t> Index(std::string_view value) {
  if (value.empty()) return {};
  uint32_t index = 0;
  const auto parsed =
      std::from_chars(value.data(), value.data() + value.size(), index);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size())
    return {};
  return index;
}
std::string_view LocalName(std::string_view name) {
  const auto colon = name.find(':');
  return colon == name.npos ? name : name.substr(colon + 1);
}
bool SwElement(pugi::xml_node node, std::string_view local) {
  const std::string_view name = node.name();
  if (LocalName(name) != local) return false;
  const auto colon = name.find(':');
  const std::string attribute =
      colon == name.npos ? "xmlns"
                         : "xmlns:" + std::string(name.substr(0, colon));
  for (auto parent = node; parent; parent = parent.parent())
    if (auto ns = parent.attribute(attribute.c_str()))
      return ns.value() == kSwNamespace;
  return false;
}
bool Keywords(std::string_view name) {
  return name == "swXmlContents/KeyWords" || name == "swXmlContents/Keywords" ||
         name == "Keywords";
}
bool Features(std::string_view name) {
  return name == "swXmlContents/Features" || name == "Features";
}
struct Metadata {
  std::set<uint32_t> partitions;
  std::set<uint32_t> slots;
  std::set<bool> active;
  std::set<bool> rebuild;
  bool named = false;
  bool conflict = false;
};
void Evidence(SavedGeometryResolution& result, const StreamInfo& stream,
              std::string field, std::string value) {
  result.evidence.push_back({{stream.name, stream.marker_offset, 0},
                             std::move(field),
                             std::move(value)});
}
void Problem(SavedGeometryResolution& result, Metadata& metadata,
             std::string message) {
  metadata.conflict = true;
  result.limitations.push_back(std::move(message));
}
void ReadKeywords(const std::vector<pugi::xml_node>& nodes,
                  const StreamInfo& stream, SavedGeometryResolution& result,
                  Metadata& metadata) {
  unsigned matches = 0;
  for (const auto node : nodes) {
    if (node.parent() != nodes.front() ||
        std::string_view(node.name()) != "Configuration" ||
        node.attribute("Name").value() != result.configuration)
      continue;
    ++matches;
    metadata.named = true;
    for (const auto field : {"SourceIndex", "id"}) {
      const auto attribute = node.attribute(field);
      if (!attribute) continue;
      Evidence(result, stream, std::string("Keywords.Configuration.") + field,
               attribute.value());
      const auto index = Index(attribute.value());
      if (!index) {
        Problem(result, metadata,
                std::string("Invalid Keywords configuration ") + field);
      } else if (std::string_view(field) == "SourceIndex") {
        metadata.partitions.insert(*index);
      } else {
        metadata.slots.insert(*index);
      }
    }
  }
  if (matches > 1)
    Problem(result, metadata, "Duplicate named Keywords configurations");
}
void ReadFeatures(const std::vector<pugi::xml_node>& nodes,
                  const StreamInfo& stream, SavedGeometryResolution& result,
                  Metadata& metadata) {
  std::map<std::string, std::vector<pugi::xml_node>> models;
  std::vector<pugi::xml_node> rows;
  unsigned active_count = 0;
  for (const auto node : nodes) {
    if (SwElement(node, "swModel") && SwElement(node.parent(), "swModelList") &&
        node.parent().parent() == nodes.front())
      models[node.attribute("id").value()].push_back(node);
    if (SwElement(node, "swConfiguration") &&
        SwElement(node.parent(), "swConfigurationList") &&
        node.parent().parent() == nodes.front()) {
      rows.push_back(node);
      if (std::string_view(
              node.attribute("swMostRecentConfiguration").value()) == "YES")
        ++active_count;
    }
  }
  unsigned matches = 0;
  for (const auto row : rows) {
    const auto found = models.find(row.attribute("swModelRef").value());
    const bool row_names_target =
        row.attribute("swName").value() == result.configuration;
    if (found == models.end() || found->first.empty() ||
        found->second.size() != 1) {
      if (row_names_target || !row.attribute("swName"))
        Problem(result, metadata,
                "Features configuration has an absent or ambiguous swModelRef");
      continue;
    }
    const auto model = found->second.front();
    if (model.attribute("swConfigurationName").value() !=
        result.configuration) {
      if (row_names_target)
        Problem(result, metadata,
                "Features configuration/model names disagree");
      continue;
    }
    ++matches;
    metadata.named = true;
    Evidence(result, stream, "Features.swModelRef",
             row.attribute("swModelRef").value());
    Evidence(result, stream, "Features.swConfiguration.swID",
             row.attribute("swID").value());
    const auto index = Index(row.attribute("swID").value());
    const auto model_index =
        Index(model.attribute("swConfigurationId").value());
    if (!index || !model_index || index != model_index ||
        (row.attribute("swName") && !row_names_target)) {
      Problem(result, metadata,
              "Features configuration swID/model identity mismatch");
      continue;
    }
    metadata.partitions.insert(*index);
    if (active_count == 1) {
      const std::string_view active =
          row.attribute("swMostRecentConfiguration").value();
      if (active == "YES" || active == "NO")
        metadata.active.insert(active == "YES");
    }
    const std::string_view rebuild =
        row.attribute("swConfigurationNeedsUpdate").value();
    if (rebuild == "YES" || rebuild == "NO")
      metadata.rebuild.insert(rebuild == "YES");
    if (row.attribute("swMostRecentConfiguration"))
      Evidence(result, stream, "Features.swMostRecentConfiguration",
               row.attribute("swMostRecentConfiguration").value());
    if (row.attribute("swConfigurationNeedsUpdate"))
      Evidence(result, stream, "Features.swConfigurationNeedsUpdate",
               std::string(rebuild));
  }
  if (matches > 1)
    Problem(result, metadata, "Duplicate Features configuration identity");
  if (active_count != 1)
    result.limitations.push_back(
        "Features manifest does not identify one last-active configuration");
}
std::optional<std::string> ReadMetadata(const ContainerInspection& inspection,
                                        SavedGeometryResolution& result,
                                        Metadata& metadata,
                                        const SavedGeometryOptions& options) {
  uint64_t xml_bytes = 0, xml_nodes = 0;
  unsigned keyword_streams = 0, feature_streams = 0;
  for (const auto& stream : inspection.streams) {
    const bool keywords = Keywords(stream.name),
               features = Features(stream.name);
    if (!keywords && !features) continue;
    Evidence(result, stream, "selector_stream.status", ToString(stream.status));
    Evidence(result, stream, "selector_stream.primary",
             stream.selected_as_primary ? "true" : "false");
    if ((keywords && ++keyword_streams > 1) ||
        (features && ++feature_streams > 1))
      Problem(result, metadata,
              "Multiple configuration-selector streams; physical order is not "
              "authoritative");
    if (stream.status != StreamDecodeStatus::kDecoded ||
        !stream.selected_as_primary) {
      Problem(result, metadata,
              "Unavailable or duplicate configuration-selector stream: " +
                  stream.name);
      continue;
    }
    if (stream.decoded_data.size() > options.max_xml_bytes - xml_bytes)
      return "Saved geometry XML byte limit exceeded";
    xml_bytes += stream.decoded_data.size();
    auto bytes = std::span<const uint8_t>(stream.decoded_data);
    // Native KeyWords may carry this one-byte serialization tag before UTF-8
    // XML.
    if (keywords && !bytes.empty() && bytes.front() == 0x86)
      bytes = bytes.subspan(1);
    pugi::xml_document xml;
    if (!xml.load_buffer(bytes.data(), bytes.size(), pugi::parse_default)) {
      Problem(result, metadata,
              "Malformed configuration-selector XML: " + stream.name);
      continue;
    }
    const auto root = xml.document_element();
    if ((features && !SwElement(root, "swSolidWorks")) ||
        (keywords && std::string_view(root.name()) != "Keywords")) {
      Problem(result, metadata,
              "Unsupported configuration-selector root or namespace: " +
                  stream.name);
      continue;
    }
    std::vector<pugi::xml_node> nodes;
    for (auto node = root; node;) {
      if (++xml_nodes > options.max_xml_nodes)
        return "Saved geometry XML node limit exceeded";
      if (node.type() == pugi::node_element) nodes.push_back(node);
      if (node.first_child()) {
        node = node.first_child();
        continue;
      }
      while (node && node != root && !node.next_sibling()) node = node.parent();
      if (!node || node == root) break;
      node = node.next_sibling();
    }
    if (features)
      ReadFeatures(nodes, stream, result, metadata);
    else
      ReadKeywords(nodes, stream, result, metadata);
  }
  return {};
}
std::string_view ConfigSuffix(std::string_view stream,
                              std::optional<uint32_t>& index) {
  if (stream.starts_with("Contents/")) stream.remove_prefix(9);
  if (!stream.starts_with("Config-")) return {};
  stream.remove_prefix(7);
  const auto dash = stream.find('-');
  if (dash == stream.npos) return {};
  index = Index(stream.substr(0, dash));
  return index ? stream.substr(dash + 1) : std::string_view{};
}
GeometrySourceRole Role(const ParasolidPayload& payload,
                        std::optional<uint32_t>& index) {
  const auto suffix = ConfigSuffix(payload.source_stream, index);
  if (suffix == "GhostPartition") return GeometrySourceRole::kGhost;
  if (suffix == "FeatureBodies/LocalBodies" || suffix == "ResolvedFeatures")
    return GeometrySourceRole::kInternalFeature;
  if (suffix == "Partition" && payload.kind == ParasolidPayloadKind::kPartition)
    return GeometrySourceRole::kPartition;
  if (payload.kind == ParasolidPayloadKind::kDeltas)
    return GeometrySourceRole::kDelta;
  return GeometrySourceRole::kOther;
}
std::optional<std::string> DecodeBodies(const ParasolidPayload& payload,
                                        SavedGeometryCandidate& candidate,
                                        internal::SavedHistoryData& history,
                                        const SavedGeometryOptions& options) {
  openparasolid::SchemaLimits limits;
  limits.max_input_bytes = options.max_decoded_bytes;
  limits.max_nodes = options.max_nodes_per_payload;
  limits.max_delta_entries = options.max_nodes_per_payload;
  limits.max_delta_packets = options.max_nodes_per_payload;
  auto parsed =
      openparasolid::SchemaParser::Parse(payload.payload.data, nullptr, limits);
  if (!parsed.ok()) {
    if (parsed.error().code == openparasolid::ErrorCode::kResourceLimit)
      return parsed.error().message;
    candidate.decode_issue = parsed.error().message;
    return {};
  }
  for (const auto& diagnostic : parsed.value().diagnostics)
    if (diagnostic.error.code == openparasolid::ErrorCode::kResourceLimit)
      return diagnostic.error.message;
  if (!parsed.value().complete || !parsed.value().diagnostics.empty()) {
    candidate.decode_issue = "Incomplete schema decoding; BODY list withheld";
    return {};
  }
  if (payload.kind == ParasolidPayloadKind::kDeltas) {
    candidate.decoded_complete = true;
    history.complete = true;
    history.prefix = parsed.value().delta_prefix;
    for (const auto& packet : parsed.value().delta_packets)
      for (const auto& occurrence : packet.marks)
        history.marks.push_back(occurrence.mark);
    return {};
  }
  // WORLD current_id is a signed wire word in the generic schema report.
  // Preserve its 32 bits; an unset sentinel is not an identifier.
  const openparasolid::DecodedNode* world = nullptr;
  bool multiple_worlds = false;
  for (const auto& node : parsed.value().nodes)
    if (node.node_type == 101) {
      if (world) multiple_worlds = true;
      world = &node;
    }
  if (world && !multiple_worlds) {
    const auto& definition =
        parsed.value().schemas[world->schema_index].definition;
    std::size_t current_fields = 0;
    for (const auto& field : world->fields) {
      const auto& field_definition = definition.fields[field.definition_index];
      if (field_definition.name != "current_id") continue;
      ++current_fields;
      if (field_definition.type == openparasolid::FieldType::kInteger &&
          field_definition.elements == 0 && field.values.size() == 1) {
        const auto* value = std::get_if<int64_t>(&field.values.front());
        if (value && *value >= std::numeric_limits<int32_t>::min() &&
            *value <= std::numeric_limits<int32_t>::max())
          candidate.world_current_id = static_cast<uint32_t>(*value);
      }
    }
    if (current_fields != 1) candidate.world_current_id.reset();
  }
  openparasolid::ModelOptions model_options;
  model_options.max_nodes = options.max_nodes_per_payload;
  auto model =
      openparasolid::ModelBuilder::Build(parsed.value(), model_options);
  if (!model.ok()) {
    if (model.error().code == openparasolid::ErrorCode::kResourceLimit)
      return model.error().message;
    candidate.decode_issue = model.error().message;
    return {};
  }
  candidate.decoded_complete = true;
  for (const auto& node : model.value().nodes()) {
    if (node.node_type != 12) continue;
    SavedBodyKind kind = SavedBodyKind::kUnknown;
    switch (node.body_type.value_or(0)) {
      case 1:
        kind = SavedBodyKind::kSolid;
        break;
      case 2:
        kind = SavedBodyKind::kWire;
        break;
      case 3:
        kind = SavedBodyKind::kSheet;
        break;
      case 6:
        kind = SavedBodyKind::kGeneral;
        break;
    }
    candidate.bodies.push_back({node.id.value, kind});
  }
  return {};
}
}  // namespace
Result<SavedGeometryResolution> ResolveSavedGeometry(
    const ContainerInspection& inspection, std::string_view configuration,
    const SavedGeometryOptions& options) {
  using Return = Result<SavedGeometryResolution>;
  if (configuration.empty() || !options.max_streams ||
      !options.max_decoded_bytes || !options.max_xml_bytes ||
      !options.max_xml_nodes || !options.max_payloads ||
      !options.max_nodes_per_payload)
    return Return::Err(
        "Saved geometry requires a configuration and positive resource limits");
  if (inspection.streams.size() > options.max_streams)
    return Return::Err("Saved geometry stream limit exceeded");
  uint64_t total = 0;
  for (const auto& stream : inspection.streams) {
    if (stream.decoded_data.size() > options.max_decoded_bytes - total)
      return Return::Err("Saved geometry decoded byte limit exceeded");
    total += stream.decoded_data.size();
  }
  SavedGeometryResolution result;
  result.configuration = configuration;
  Metadata metadata;
  if (const auto error = ReadMetadata(inspection, result, metadata, options))
    return Return::Err(*error);
  if (metadata.partitions.size() > 1 || metadata.slots.size() > 1 ||
      metadata.active.size() > 1 || metadata.rebuild.size() > 1)
    Problem(result, metadata,
            "Conflicting configuration identities or cache flags");
  if (!metadata.conflict && metadata.partitions.size() == 1) {
    result.configuration_status = GeometryResolutionStatus::kResolved;
    result.partition_index = *metadata.partitions.begin();
  }
  if (!metadata.conflict && metadata.slots.size() == 1)
    result.resolved_features_index = *metadata.slots.begin();
  if (!metadata.conflict && metadata.active.size() == 1)
    result.was_last_active = *metadata.active.begin();
  if (!metadata.conflict && metadata.rebuild.size() == 1)
    result.needs_rebuild = *metadata.rebuild.begin();
  const auto extraction = ExtractParasolidStreams(inspection);
  if (extraction.payloads.size() > options.max_payloads)
    return Return::Err("Saved geometry payload limit exceeded");
  if (!extraction.issues.empty())
    result.limitations.push_back(
        "Payload extraction has issues; complete saved state cannot be "
        "established");
  std::vector<std::size_t> matching, partitions;
  std::vector<internal::SavedHistoryData> histories(extraction.payloads.size());
  for (std::size_t i = 0; i < extraction.payloads.size(); ++i) {
    const auto& payload = extraction.payloads[i];
    SavedGeometryCandidate candidate;
    candidate.payload_index = i;
    candidate.source = {payload.source_stream, payload.container_marker_offset,
                        payload.payload.marker_offset};
    candidate.kind = payload.kind;
    candidate.role = Role(payload, candidate.stream_index);
    if (candidate.role == GeometrySourceRole::kPartition) {
      partitions.push_back(i);
      if (result.partition_index &&
          candidate.stream_index == result.partition_index)
        matching.push_back(i);
    }
    if (const auto error =
            DecodeBodies(payload, candidate, histories[i], options))
      return Return::Err(*error);
    result.candidates.push_back(std::move(candidate));
  }
  bool duplicate_partition = false;
  for (const auto& stream : inspection.streams) {
    std::optional<uint32_t> index;
    if (ConfigSuffix(stream.name, index) != "Partition") continue;
    if ((!stream.selected_as_primary ||
         stream.status != StreamDecodeStatus::kDecoded) &&
        (!result.partition_index || index == result.partition_index)) {
      duplicate_partition = true;
      Evidence(result, stream, "partition_stream.primary",
               stream.selected_as_primary ? "true" : "false");
      Evidence(result, stream, "partition_stream.status",
               ToString(stream.status));
    }
  }
  if (duplicate_partition)
    result.limitations.push_back(
        "Duplicate or unreadable partition chunks prevent a physical-order "
        "selection");
  if (matching.size() == 1 && !metadata.conflict && !duplicate_partition) {
    result.partition_status = GeometryResolutionStatus::kResolved;
    result.candidates[matching.front()].selected_partition = true;
  } else if (!result.partition_index && !metadata.conflict &&
             !duplicate_partition && metadata.named && partitions.size() == 1) {
    result.partition_status = GeometryResolutionStatus::kHeuristic;
    result.candidates[partitions.front()].selected_partition = true;
    result.limitations.push_back(
        "Sole configuration partition is a heuristic; no stored partition "
        "selector was found");
  } else {
    result.limitations.push_back(
        "No unique configuration partition selected; internal feature payloads "
        "are not substitutes");
  }
  if (matching.size() > 1)
    result.limitations.push_back(
        "Multiple partition payloads match the selector; no physical-order "
        "choice was made");
  for (const auto& candidate : result.candidates)
    if (candidate.selected_partition) {
      const bool delta =
          std::any_of(result.candidates.begin(), result.candidates.end(),
                      [&](const auto& other) {
                        return other.kind == ParasolidPayloadKind::kDeltas &&
                               other.source.stream == candidate.source.stream &&
                               other.source.container_offset ==
                                   candidate.source.container_offset;
                      });
      if (delta) {
        result.state_status = GeometryResolutionStatus::kUnsupported;
        result.limitations.push_back(
            "Associated delta candidates exist; automatic "
            "base/history/operation resolution is not implemented");
      }
    }
  result.limitations.push_back(
      "Partition identity does not certify the saved target state or complete "
      "final-body membership");
  result.limitations.push_back(
      "BODY lists are per-payload candidates; no cross-payload union or "
      "final/helper role classification is performed");
  if (result.needs_rebuild.value_or(false))
    result.limitations.push_back(
        "Selected configuration cache is marked as needing rebuild");
  internal::InspectSavedState(result, histories);
  if (result.state_evidence.relation ==
      SavedStateRelation::kMatchingIdentifiers)
    result.limitations.push_back(
        "Baseline WORLD and delta target identifiers match; delta presence "
        "alone does not require replay or certify final-body membership");
  return Return::Ok(std::move(result));
}
Result<SavedGeometryResolution> ResolveSavedGeometry(
    const std::filesystem::path& path, std::string_view configuration,
    const SavedGeometryOptions& options) {
  auto inspected = InspectContainer(path);
  if (!inspected.ok())
    return Result<SavedGeometryResolution>::Err(inspected.error());
  return ResolveSavedGeometry(inspected.value(), configuration, options);
}
const char* ToString(GeometryResolutionStatus value) noexcept {
  switch (value) {
    case GeometryResolutionStatus::kResolved:
      return "resolved";
    case GeometryResolutionStatus::kHeuristic:
      return "heuristic";
    case GeometryResolutionStatus::kUnresolved:
      return "unresolved";
    case GeometryResolutionStatus::kUnsupported:
      return "unsupported";
  }
  return "unresolved";
}
const char* ToString(GeometrySourceRole value) noexcept {
  switch (value) {
    case GeometrySourceRole::kPartition:
      return "configuration_partition";
    case GeometrySourceRole::kDelta:
      return "delta";
    case GeometrySourceRole::kInternalFeature:
      return "internal_feature_geometry";
    case GeometrySourceRole::kGhost:
      return "ghost_partition";
    case GeometrySourceRole::kOther:
      return "other";
  }
  return "other";
}
const char* ToString(SavedBodyKind value) noexcept {
  switch (value) {
    case SavedBodyKind::kSolid:
      return "solid";
    case SavedBodyKind::kSheet:
      return "sheet";
    case SavedBodyKind::kWire:
      return "wire";
    case SavedBodyKind::kGeneral:
      return "general";
    case SavedBodyKind::kUnknown:
      return "unknown";
  }
  return "unknown";
}
const char* ToString(SavedStateRelation value) noexcept {
  switch (value) {
    case SavedStateRelation::kUnavailable:
      return "unavailable";
    case SavedStateRelation::kMatchingIdentifiers:
      return "matching_identifiers";
    case SavedStateRelation::kDifferentIdentifiers:
      return "different_identifiers";
  }
  return "unavailable";
}
}  // namespace openswx
