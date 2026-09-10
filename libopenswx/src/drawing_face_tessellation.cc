// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/drawing_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace openswx {
namespace {

constexpr std::string_view kFaceTessellationClass = "uoTempFaceTessData_c";
constexpr std::size_t kDescriptorCount = 6u;
constexpr uint32_t kMaximumItems = 20'000'000u;

uint32_t ReadU32Le(std::span<const uint8_t> data, std::size_t offset) {
  return static_cast<uint32_t>(data[offset]) |
         (static_cast<uint32_t>(data[offset + 1u]) << 8u) |
         (static_cast<uint32_t>(data[offset + 2u]) << 16u) |
         (static_cast<uint32_t>(data[offset + 3u]) << 24u);
}

float ReadF32Le(std::span<const uint8_t> data, std::size_t offset) {
  const uint32_t bits = ReadU32Le(data, offset);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool Has(std::span<const uint8_t> data, std::size_t offset,
         std::size_t length) {
  return offset <= data.size() && length <= data.size() - offset;
}

struct Descriptor {
  uint32_t item_size = 0;
  uint32_t kind = 0;
  uint32_t flags = 0;
  uint32_t count = 0;
  std::size_t payload_offset = 0;
};

struct ParsedTable {
  FaceTessellation mesh;
  std::size_t end = 0;
};

struct ModelReference {
  std::size_t offset = 0;
  std::string stem;
};

uint16_t ReadU16Le(std::span<const uint8_t> data, std::size_t offset) {
  return static_cast<uint16_t>(data[offset]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[offset + 1u]) << 8u);
}

void AppendUtf8(std::string& output, uint16_t codepoint) {
  if (codepoint <= 0x7Fu) {
    output.push_back(static_cast<char>(codepoint));
  } else if (codepoint <= 0x7FFu) {
    output.push_back(static_cast<char>(0xC0u | (codepoint >> 6u)));
    output.push_back(static_cast<char>(0x80u | (codepoint & 0x3Fu)));
  } else {
    output.push_back(static_cast<char>(0xE0u | (codepoint >> 12u)));
    output.push_back(static_cast<char>(0x80u | ((codepoint >> 6u) & 0x3Fu)));
    output.push_back(static_cast<char>(0x80u | (codepoint & 0x3Fu)));
  }
}

char AsciiLower(char value) {
  return value >= 'A' && value <= 'Z'
             ? static_cast<char>(value - 'A' + 'a')
             : value;
}

bool EndsWithAsciiCaseInsensitive(std::string_view value,
                                  std::string_view suffix) {
  if (value.size() < suffix.size()) return false;
  value.remove_prefix(value.size() - suffix.size());
  return std::equal(value.begin(), value.end(), suffix.begin(), suffix.end(),
                    [](char left, char right) {
                      return AsciiLower(left) == AsciiLower(right);
                    });
}

std::vector<ModelReference> FindModelReferences(
    std::span<const uint8_t> data) {
  std::vector<ModelReference> references;
  for (std::size_t offset = 0; offset + 4u <= data.size(); ++offset) {
    if (data[offset] != 0xFFu || data[offset + 1u] != 0xFEu ||
        data[offset + 2u] != 0xFFu) {
      continue;
    }
    const std::size_t units = data[offset + 3u];
    const std::size_t start = offset + 4u;
    if (units == 0u || start > data.size() ||
        units > (data.size() - start) / 2u) {
      continue;
    }
    std::string value;
    value.reserve(units);
    bool valid = true;
    for (std::size_t index = 0; index < units; ++index) {
      const uint16_t codepoint = ReadU16Le(data, start + index * 2u);
      if (codepoint == 0u ||
          (codepoint >= 0xD800u && codepoint <= 0xDFFFu)) {
        valid = false;
        break;
      }
      AppendUtf8(value, codepoint);
    }
    if (!valid ||
        (!EndsWithAsciiCaseInsensitive(value, ".sldprt") &&
         !EndsWithAsciiCaseInsensitive(value, ".sldasm"))) {
      continue;
    }
    const std::size_t separator = value.find_last_of("/\\");
    std::string filename = value.substr(
        separator == std::string::npos ? 0u : separator + 1u);
    const std::size_t extension = filename.find_last_of('.');
    if (extension != std::string::npos) filename.resize(extension);
    if (!filename.empty()) references.push_back({offset, std::move(filename)});
  }
  return references;
}

const ModelReference* ModelReferenceAt(
    std::span<const ModelReference> references,
    std::size_t offset) {
  const auto owner = std::upper_bound(
      references.begin(), references.end(), offset,
      [](std::size_t value, const ModelReference& reference) {
        return value < reference.offset;
      });
  return owner == references.begin() ? nullptr : &*std::prev(owner);
}

std::optional<ParsedTable> ParseTable(std::span<const uint8_t> data,
                                      std::size_t table_offset,
                                      std::size_t limit) {
  if (limit > data.size() || table_offset >= limit) return std::nullopt;

  std::array<Descriptor, kDescriptorCount> descriptors{};
  std::size_t cursor = table_offset;
  for (Descriptor& descriptor : descriptors) {
    if (cursor > limit || limit - cursor < 16u) return std::nullopt;
    descriptor.item_size = ReadU32Le(data, cursor);
    descriptor.kind = ReadU32Le(data, cursor + 4u);
    descriptor.flags = ReadU32Le(data, cursor + 8u);
    descriptor.count = ReadU32Le(data, cursor + 12u);
    descriptor.payload_offset = cursor + 16u;
    if (descriptor.item_size == 0u || descriptor.count > kMaximumItems) {
      return std::nullopt;
    }
    const uint64_t payload_size =
        static_cast<uint64_t>(descriptor.item_size) * descriptor.count;
    if (payload_size > static_cast<uint64_t>(limit - descriptor.payload_offset)) {
      return std::nullopt;
    }
    cursor = descriptor.payload_offset + static_cast<std::size_t>(payload_size);
  }

  const auto signature = [&](std::size_t index, uint32_t item_size,
                             uint32_t kind) {
    const Descriptor& descriptor = descriptors[index];
    return descriptor.item_size == item_size && descriptor.kind == kind &&
           descriptor.flags == 2u;
  };
  if (!signature(0u, 4u, 8u) || !signature(1u, 12u, 100u) ||
      !signature(2u, 12u, 100u) || !signature(3u, 4u, 8u) ||
      !signature(4u, 4u, 8u) || !signature(5u, 1u, 8u)) {
    return std::nullopt;
  }

  ParsedTable parsed;
  parsed.end = cursor;
  parsed.mesh.source_table_offset = table_offset;
  parsed.mesh.strip_lengths.reserve(descriptors[0].count);
  uint64_t vertex_count_from_strips = 0u;
  uint64_t endpoint_count = 0u;
  for (uint32_t index = 0; index < descriptors[0].count; ++index) {
    const uint32_t length =
        ReadU32Le(data, descriptors[0].payload_offset + index * 4u);
    if (length < 3u) return std::nullopt;
    parsed.mesh.strip_lengths.push_back(length);
    vertex_count_from_strips += length;
    endpoint_count += static_cast<uint64_t>(length) * 2u - 2u;
  }
  if (vertex_count_from_strips != descriptors[1].count ||
      (descriptors[2].count != 0u &&
       descriptors[2].count != descriptors[1].count) ||
      endpoint_count > std::numeric_limits<uint32_t>::max()) {
    return std::nullopt;
  }

  if (descriptors[4].count != descriptors[0].count) return std::nullopt;
  for (uint32_t index = 0; index < descriptors[4].count; ++index) {
    const uint32_t expected = parsed.mesh.strip_lengths[index] * 2u - 2u;
    if (ReadU32Le(data, descriptors[4].payload_offset + index * 4u) != expected) {
      return std::nullopt;
    }
  }
  const bool no_endpoint_data =
      descriptors[3].count == 0u && descriptors[5].count == 0u;
  const bool complete_endpoint_data =
      descriptors[3].count == endpoint_count &&
      descriptors[5].count == endpoint_count;
  if (!no_endpoint_data && !complete_endpoint_data) return std::nullopt;
  for (uint32_t index = 0; index < descriptors[3].count; ++index) {
    if (!std::isfinite(ReadF32Le(
            data, descriptors[3].payload_offset + index * 4u))) {
      return std::nullopt;
    }
  }

  parsed.mesh.vertices.reserve(descriptors[1].count);
  for (uint32_t index = 0; index < descriptors[1].count; ++index) {
    const std::size_t offset = descriptors[1].payload_offset + index * 12u;
    const Vec3 vertex{ReadF32Le(data, offset), ReadF32Le(data, offset + 4u),
                      ReadF32Le(data, offset + 8u)};
    if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) ||
        !std::isfinite(vertex.z)) {
      return std::nullopt;
    }
    parsed.mesh.vertices.push_back(vertex);
  }
  parsed.mesh.normals.reserve(descriptors[2].count);
  for (uint32_t index = 0; index < descriptors[2].count; ++index) {
    const std::size_t offset = descriptors[2].payload_offset + index * 12u;
    const Vec3 normal{ReadF32Le(data, offset), ReadF32Le(data, offset + 4u),
                      ReadF32Le(data, offset + 8u)};
    if (!std::isfinite(normal.x) || !std::isfinite(normal.y) ||
        !std::isfinite(normal.z)) {
      return std::nullopt;
    }
    parsed.mesh.normals.push_back(normal);
  }

  uint32_t base = 0u;
  for (const uint32_t length : parsed.mesh.strip_lengths) {
    for (uint32_t index = 0; index + 2u < length; ++index) {
      if ((index & 1u) == 0u) {
        parsed.mesh.triangles.push_back(
            {base + index, base + index + 1u, base + index + 2u});
      } else {
        parsed.mesh.triangles.push_back(
            {base + index, base + index + 2u, base + index + 1u});
      }
    }
    base += length;
  }
  return parsed;
}

std::vector<std::size_t> FindClassOffsets(std::span<const uint8_t> data) {
  std::vector<std::size_t> offsets;
  const auto begin = data.begin();
  auto cursor = begin;
  while (cursor != data.end()) {
    cursor = std::search(cursor, data.end(), kFaceTessellationClass.begin(),
                         kFaceTessellationClass.end());
    if (cursor == data.end()) break;
    offsets.push_back(static_cast<std::size_t>(cursor - begin));
    cursor += static_cast<std::ptrdiff_t>(kFaceTessellationClass.size());
  }
  return offsets;
}

}  // namespace

Result<std::vector<FaceTessellation>>
DrawingGeometryReader::ReadFaceTessellations(
    std::span<const uint8_t> data) {
  const std::vector<std::size_t> class_offsets = FindClassOffsets(data);
  const std::vector<ModelReference> model_references =
      FindModelReferences(data);
  if (class_offsets.empty()) {
    return Result<std::vector<FaceTessellation>>::Ok({});
  }

  std::vector<FaceTessellation> meshes;
  for (std::size_t class_index = 0; class_index < class_offsets.size();
       ++class_index) {
    const std::size_t class_offset = class_offsets[class_index];
    const std::size_t header = class_offset + kFaceTessellationClass.size();
    const std::size_t limit = class_index + 1u < class_offsets.size()
                                  ? class_offsets[class_index + 1u]
                                  : data.size();
    if (!Has(data, header, 8u) || header + 8u > limit) continue;

    const uint32_t triangle_count = ReadU32Le(data, header);
    const uint32_t strip_count = ReadU32Le(data, header + 4u);
    std::size_t table_offset = header + 8u;
    if (Has(data, header, 40u) && header + 40u <= limit &&
        ReadU32Le(data, header + 8u) == 1u &&
        ReadU32Le(data, header + 12u) == 0u &&
        ReadU32Le(data, header + 16u) == 0u &&
        ReadU32Le(data, header + 20u) != 0u &&
        std::all_of(data.begin() + static_cast<std::ptrdiff_t>(header + 24u),
                    data.begin() + static_cast<std::ptrdiff_t>(header + 40u),
                    [](uint8_t byte) { return byte == 0u; })) {
      table_offset = header + 40u;
    }

    auto first = ParseTable(data, table_offset, limit);
    if (!first || first->mesh.strip_lengths.size() != strip_count ||
        first->mesh.triangles.size() != triangle_count) {
      continue;
    }
    first->mesh.source_offset = class_offset;
    if (const ModelReference* reference =
            ModelReferenceAt(model_references, table_offset)) {
      first->mesh.referenced_document = reference->stem;
      first->mesh.source_group_offset = reference->offset;
    }
    std::size_t cursor = first->end;
    meshes.push_back(std::move(first->mesh));

    while (cursor + 4u <= limit) {
      if (ReadU32Le(data, cursor) != 4u) {
        ++cursor;
        continue;
      }
      auto table = ParseTable(data, cursor, limit);
      if (!table) {
        cursor += 4u;
        continue;
      }
      table->mesh.source_offset = class_offset;
      if (const ModelReference* reference =
              ModelReferenceAt(model_references, cursor)) {
        table->mesh.referenced_document = reference->stem;
        table->mesh.source_group_offset = reference->offset;
      }
      cursor = table->end;
      meshes.push_back(std::move(table->mesh));
    }
  }

  if (meshes.empty()) {
    return Result<std::vector<FaceTessellation>>::Err(
        "uoTempFaceTessData_c was found but no valid descriptor table decoded");
  }
  return Result<std::vector<FaceTessellation>>::Ok(std::move(meshes));
}

}  // namespace openswx
