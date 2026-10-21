// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openswx/drawing_geometry.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstring>
#include <map>
#include <optional>
#include <set>
#include <string_view>
#include <tuple>

namespace openswx {
namespace {

constexpr std::array<uint8_t, 3> kUtf16Marker = {0xFF, 0xFE, 0xFF};
constexpr uint32_t kLineOpcode = 80u;
constexpr uint32_t kTwoPointPayload = 4u;
constexpr std::size_t kLinePayloadBytes = 56u;
constexpr uint32_t kArcOpcode = 128u;
constexpr uint32_t kArcPayload = 2u;
constexpr std::size_t kArcPayloadBytes = 104u;
constexpr uint32_t kTextPayload = 18u;
constexpr std::size_t kTextPrefixBytes = 52u;
constexpr uint32_t kTextStyleOpcode = 108u;
constexpr uint32_t kTextStylePayload = 10u;
constexpr std::size_t kTextStylePrefixBytes = 54u;
constexpr std::size_t kMaximumOwnerDistance = 100000u;
constexpr std::size_t kMaximumTextStyleDistance = 2000u;
constexpr double kDefaultDimensionTextHeightM = 0.0025;
constexpr double kTwoPi = 6.28318530717958647692;

bool IsTextOpcode(uint32_t opcode) {
  // The final decimal digit identifies the text-command family; the higher
  // digits encode alignment and style variants. Drawing formats use variants
  // through 1156, while view annotations commonly use 156-186.
  return opcode >= 156u && opcode <= 1156u && opcode % 10u == 6u;
}

uint16_t ReadU16Le(std::span<const uint8_t> data, std::size_t offset) {
  return static_cast<uint16_t>(data[offset]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[offset + 1]) << 8u);
}

uint32_t ReadU32Le(std::span<const uint8_t> data, std::size_t offset) {
  return static_cast<uint32_t>(data[offset]) |
         (static_cast<uint32_t>(data[offset + 1]) << 8u) |
         (static_cast<uint32_t>(data[offset + 2]) << 16u) |
         (static_cast<uint32_t>(data[offset + 3]) << 24u);
}

double ReadF64Le(std::span<const uint8_t> data, std::size_t offset) {
  std::array<uint8_t, sizeof(double)> bytes{};
  std::copy_n(data.begin() + static_cast<std::ptrdiff_t>(offset), bytes.size(),
              bytes.begin());
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  std::reverse(bytes.begin(), bytes.end());
#endif
  double value = 0.0;
  std::memcpy(&value, bytes.data(), sizeof(value));
  return value;
}

float ReadF32Le(std::span<const uint8_t> data, std::size_t offset) {
  std::array<uint8_t, sizeof(float)> bytes{};
  std::copy_n(data.begin() + static_cast<std::ptrdiff_t>(offset), bytes.size(),
              bytes.begin());
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  std::reverse(bytes.begin(), bytes.end());
#endif
  float value = 0.0F;
  std::memcpy(&value, bytes.data(), sizeof(value));
  return value;
}

void AppendUtf8(std::string &output, uint16_t codepoint) {
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

std::optional<std::string> ReadUtf16Field(std::span<const uint8_t> data,
                                          std::size_t offset) {
  if (offset + 4u > data.size() ||
      !std::equal(kUtf16Marker.begin(), kUtf16Marker.end(),
                  data.begin() + static_cast<std::ptrdiff_t>(offset))) {
    return std::nullopt;
  }
  const std::size_t units = data[offset + 3u];
  const std::size_t start = offset + 4u;
  if (units == 0u || units == 0xFFu || units > 127u ||
      units > (data.size() - start) / 2u) {
    return std::nullopt;
  }
  std::string value;
  value.reserve(units);
  for (std::size_t index = 0; index < units; ++index) {
    const uint16_t codepoint = ReadU16Le(data, start + index * 2u);
    if (codepoint == 0u || (codepoint >= 0xD800u && codepoint <= 0xDFFFu)) {
      return std::nullopt;
    }
    AppendUtf8(value, codepoint);
  }
  return value;
}

struct ViewReference {
  std::size_t offset = 0;
  std::string name;
};

struct ObjectReference {
  std::size_t offset = 0;
  uint32_t id = 0;
};

struct TextStyleReference {
  std::size_t offset = 0;
  double height_m = 0.0;
  std::string font_family;
  bool bold = false;
};

struct SectionCalloutOwnership {
  std::size_t begin = 0;
  std::size_t end = 0;
  std::set<uint32_t> target_object_ids;
};

std::vector<ViewReference>
FindViewReferences(std::span<const uint8_t> data,
                   const std::set<std::string> &known_names) {
  std::vector<ViewReference> references;
  for (std::size_t offset = 0; offset + 4u <= data.size(); ++offset) {
    auto field = ReadUtf16Field(data, offset);
    if (!field)
      continue;
    for (const std::string &name : known_names) {
      if (field->size() > name.size() &&
          field->compare(field->size() - name.size(), name.size(), name) == 0 &&
          (*field)[field->size() - name.size() - 1u] == '@') {
        references.push_back({offset, name});
        break;
      }
    }
    offset += 3u;
  }
  return references;
}

std::vector<ViewReference>
FindExactReferences(std::span<const uint8_t> data,
                    const std::set<std::string> &known_names) {
  std::vector<ViewReference> references;
  for (std::size_t offset = 0; offset + 4u <= data.size(); ++offset) {
    const auto field = ReadUtf16Field(data, offset);
    if (field && known_names.contains(*field)) {
      references.push_back({offset, *field});
    }
    if (field)
      offset += 3u;
  }
  return references;
}

std::vector<ViewReference>
FindSheetEndReferences(std::span<const uint8_t> data,
                       std::span<const std::string> sheet_names) {
  std::vector<ViewReference> references;
  for (std::size_t offset = 0; offset + 4u <= data.size(); ++offset) {
    const auto field = ReadUtf16Field(data, offset);
    if (!field)
      continue;
    for (const std::string &sheet_name : sheet_names) {
      if (*field == sheet_name || (field->starts_with(sheet_name) &&
                                   field->size() > sheet_name.size() &&
                                   (*field)[sheet_name.size()] == '-')) {
        references.push_back({offset, sheet_name});
        break;
      }
    }
    offset += 3u;
  }
  return references;
}

const ViewReference *
FindSheetEndOwner(const std::vector<ViewReference> &references,
                  std::size_t offset) {
  const auto owner =
      std::lower_bound(references.begin(), references.end(), offset,
                       [](const ViewReference &reference, std::size_t value) {
                         return reference.offset < value;
                       });
  return owner == references.end() ? nullptr : &*owner;
}

LinePattern ReadLinePattern(std::span<const uint8_t> data,
                            std::size_t line_offset) {
  const std::size_t start = line_offset > 80u ? line_offset - 80u : 0u;
  for (std::size_t offset = start; offset + 4u <= line_offset; ++offset) {
    const auto field = ReadUtf16Field(data, offset);
    if (!field || data[offset + 3u] == 0xFFu)
      continue;
    const std::size_t encoded_size = 4u + field->size() * 2u;
    if (offset + encoded_size != line_offset)
      continue;
    if (*field == "HIDDEN")
      return LinePattern::kHidden;
    if (*field == "CENTER") {
      return LinePattern::kCenter;
    }
    if (*field == "CHAIN") {
      return LinePattern::kChain;
    }
    if (*field == "CHAIN_THICK" || *field == "CHAIN_THICK2")
      return LinePattern::kChainThick;
    if (*field == "PHANTOM")
      return LinePattern::kPhantom;
    if (*field == "STITCH")
      return LinePattern::kStitch;
    return LinePattern::kSolid;
  }
  return LinePattern::kSolid;
}

int ReadLineWeight(std::span<const uint8_t> data, std::size_t line_offset) {
  const std::size_t start = line_offset > 80u ? line_offset - 80u : 0u;
  for (std::size_t offset = start; offset + 4u <= line_offset; ++offset) {
    const auto field = ReadUtf16Field(data, offset);
    if (!field || data[offset + 3u] == 0xFFu)
      continue;
    const std::size_t encoded_size = 4u + field->size() * 2u;
    if (offset + encoded_size != line_offset)
      continue;
    // Every confirmed line/arc command with inline style is preceded by a
    // 12-byte style payload. Its middle u32 is swLineWeights_e; ten trailing
    // bytes separate the payload from the framed UTF-16 pattern name.
    if (offset < 22u || ReadU32Le(data, offset - 22u) != 12u)
      return 0;
    const uint32_t weight = ReadU32Le(data, offset - 14u);
    return weight <= 10u ? static_cast<int>(weight) : 0;
  }
  return 0;
}

LineStyle ReadLineStyle(std::span<const uint8_t> data,
                        std::size_t command_offset) {
  LineStyle style;
  style.pattern = ReadLinePattern(data, command_offset);
  style.source_line_weight = ReadLineWeight(data, command_offset);
  return style;
}

std::vector<ObjectReference>
FindObjectReferences(std::span<const uint8_t> data) {
  constexpr std::string_view kPrefix = "<objID=";
  std::vector<ObjectReference> references;
  for (std::size_t offset = 0; offset + 4u <= data.size(); ++offset) {
    const auto field = ReadUtf16Field(data, offset);
    if (!field || !field->starts_with(kPrefix) || !field->ends_with('>')) {
      continue;
    }
    const std::string_view digits(*field);
    uint32_t id = 0;
    const char *first =
        digits.data() + static_cast<std::ptrdiff_t>(kPrefix.size());
    const char *last =
        digits.data() + static_cast<std::ptrdiff_t>(digits.size() - 1u);
    const auto parsed = std::from_chars(first, last, id);
    if (parsed.ec == std::errc{} && parsed.ptr == last && id != 0u) {
      references.push_back({offset, id});
    }
    offset += 3u;
  }
  return references;
}

std::vector<TextStyleReference> FindTextStyles(std::span<const uint8_t> data) {
  std::vector<TextStyleReference> styles;
  for (std::size_t offset = 0; offset + kTextStylePrefixBytes <= data.size();
       ++offset) {
    const uint32_t opcode = ReadU32Le(data, offset);
    if ((opcode != kTextStyleOpcode && opcode != 126u) ||
        ReadU32Le(data, offset + 4u) != kTextStylePayload) {
      continue;
    }
    double height = ReadF64Le(data, offset + 8u);
    if (height == -1.0) {
      const uint32_t points = ReadU32Le(data, offset + 16u);
      if (points == 0u || points > 72u)
        continue;
      height = static_cast<double>(points) * 0.0254 / 72.0 * 0.75;
    }
    const auto style_name = ReadUtf16Field(data, offset + 40u);
    if (!style_name || style_name->empty() || !std::isfinite(height) ||
        height < 0.0001 || height > 0.02) {
      continue;
    }
    styles.push_back({offset, height, *style_name,
                      (ReadU32Le(data, offset + 36u) & 8u) != 0u});
    offset += kTextStylePrefixBytes - 1u;
  }
  return styles;
}

const TextStyleReference *
FindTextStyle(const std::vector<TextStyleReference> &styles,
              std::size_t offset) {
  const auto next = std::upper_bound(
      styles.begin(), styles.end(), offset,
      [](std::size_t value, const TextStyleReference &reference) {
        return value < reference.offset;
      });
  if (next == styles.begin())
    return nullptr;
  const auto &style = *std::prev(next);
  return offset - style.offset <= kMaximumTextStyleDistance ? &style : nullptr;
}

std::optional<double> ReadTextRenderedWidth(std::span<const uint8_t> data,
                                            std::size_t field_offset) {
  if (field_offset + 4u > data.size())
    return std::nullopt;
  const std::size_t units = data[field_offset + 3u];
  const std::size_t suffix = field_offset + 4u + units * 2u;
  if (units == 0u || units == 0xFFu || suffix + 6u > data.size() ||
      ReadU16Le(data, suffix) != 1u ||
      ReadU32Le(data, suffix + 2u) != units ||
      units > (data.size() - suffix - 6u) / sizeof(float)) {
    return std::nullopt;
  }

  double width = 0.0;
  for (std::size_t index = 0; index < units; ++index) {
    const float advance =
        ReadF32Le(data, suffix + 6u + index * sizeof(float));
    if (!std::isfinite(advance) || advance < 0.0F || advance > 1.0F)
      return std::nullopt;
    width += advance;
  }
  return width > 0.0 && width <= 2.0 ? std::optional<double>(width)
                                     : std::nullopt;
}

bool IsSheetCoordinate(double value, double maximum) {
  return std::isfinite(value) && value >= 0.0 && value <= maximum;
}

const ViewReference *FindOwner(const std::vector<ViewReference> &references,
                               std::size_t offset) {
  const auto owner =
      std::upper_bound(references.begin(), references.end(), offset,
                       [](std::size_t value, const ViewReference &reference) {
                         return value < reference.offset;
                       });
  if (owner == references.begin())
    return nullptr;
  const ViewReference &reference = *std::prev(owner);
  if (offset - reference.offset > kMaximumOwnerDistance)
    return nullptr;
  return &reference;
}

const ObjectReference *
FindObjectOwner(const std::vector<ObjectReference> &references,
                std::size_t offset) {
  const auto owner =
      std::upper_bound(references.begin(), references.end(), offset,
                       [](std::size_t value, const ObjectReference &reference) {
                         return value < reference.offset;
                       });
  if (owner == references.begin())
    return nullptr;
  const ObjectReference &reference = *std::prev(owner);
  if (offset - reference.offset > kMaximumOwnerDistance)
    return nullptr;
  return &reference;
}

std::vector<SectionCalloutOwnership> FindSectionCalloutOwnership(
    std::span<const uint8_t> data,
    const std::vector<ObjectReference> &object_references) {
  std::vector<std::size_t> boundaries;
  for (std::size_t offset = 0; offset + 4u <= data.size(); ++offset) {
    const auto field = ReadUtf16Field(data, offset);
    if (!field)
      continue;
    if (field->starts_with("Display State-") ||
        field->starts_with("Stav zobrazení-")) {
      boundaries.push_back(offset);
    }
    offset += 3u;
  }
  boundaries.push_back(data.size());

  std::vector<SectionCalloutOwnership> ownership;
  for (std::size_t index = 0; index + 1u < boundaries.size(); ++index) {
    const std::size_t begin = boundaries[index];
    const std::size_t end = boundaries[index + 1u];
    bool has_cutting_line_pattern = false;
    for (std::size_t offset = begin; offset + 4u <= end; ++offset) {
      const auto field = ReadUtf16Field(data, offset);
      if (field && (*field == "CHAIN_THICK" || *field == "CHAIN_THICK2")) {
        has_cutting_line_pattern = true;
        break;
      }
    }
    if (!has_cutting_line_pattern)
      continue;
    std::map<char, std::size_t> letters;
    for (std::size_t offset = begin; offset + kTextPrefixBytes <= end;
         ++offset) {
      if (!IsTextOpcode(ReadU32Le(data, offset)) ||
          ReadU32Le(data, offset + 4u) != kTextPayload) {
        continue;
      }
      const auto value = ReadUtf16Field(data, offset + 48u);
      if (value && value->size() == 1u && (*value)[0] >= 'A' &&
          (*value)[0] <= 'Z') {
        ++letters[(*value)[0]];
      }
    }
    std::set<uint32_t> targets;
    for (const auto [letter, count] : letters) {
      if (count < 2u)
        continue;
      const std::string label{letter, '-', letter};
      const std::size_t search_end = std::min(data.size(), end + 100000u);
      for (std::size_t offset = end; offset + kTextPrefixBytes <= search_end;
           ++offset) {
        if (!IsTextOpcode(ReadU32Le(data, offset)) ||
            ReadU32Le(data, offset + 4u) != kTextPayload) {
          continue;
        }
        const auto value = ReadUtf16Field(data, offset + 48u);
        if (!value || *value != label)
          continue;
        if (const ObjectReference *object =
                FindObjectOwner(object_references, offset)) {
          targets.insert(object->id);
        }
        break;
      }
    }
    if (!targets.empty())
      ownership.push_back({begin, end, std::move(targets)});
  }
  return ownership;
}

std::optional<std::string>
SelectOwnerLabel(std::size_t offset,
                 const std::vector<ObjectReference> &object_references,
                 const std::set<uint32_t> &selected_object_ids,
                 const std::vector<ViewReference> &view_references,
                 const std::set<std::string> &selected_view_names,
                 const std::vector<SectionCalloutOwnership> &section_callouts) {
  const auto section =
      std::find_if(section_callouts.begin(), section_callouts.end(),
                   [&](const SectionCalloutOwnership &candidate) {
                     return offset >= candidate.begin && offset < candidate.end;
                   });
  if (section != section_callouts.end()) {
    const auto target = std::find_first_of(
        section->target_object_ids.begin(), section->target_object_ids.end(),
        selected_object_ids.begin(), selected_object_ids.end());
    if (target == section->target_object_ids.end())
      return std::nullopt;
    return "section-callout=" + std::to_string(*target);
  }
  if (!selected_object_ids.empty()) {
    if (const ObjectReference *object =
            FindObjectOwner(object_references, offset)) {
      if (!selected_object_ids.contains(object->id))
        return std::nullopt;
      return "objID=" + std::to_string(object->id);
    }
  }
  const ViewReference *view = FindOwner(view_references, offset);
  if (view == nullptr || !selected_view_names.contains(view->name)) {
    return std::nullopt;
  }
  return view->name;
}

} // namespace

Result<std::vector<Line2D>> DrawingGeometryReader::ReadDisplayListLines(
    std::span<const uint8_t> display_lists,
    std::span<const std::string> all_view_names,
    std::span<const std::string> selected_view_names, double sheet_width_m,
    double sheet_height_m, std::span<const uint32_t> selected_object_ids,
    std::span<const std::string> all_sheet_names,
    std::string_view selected_sheet_name) {
  if (!std::isfinite(sheet_width_m) || !std::isfinite(sheet_height_m) ||
      sheet_width_m <= 0.0 || sheet_height_m <= 0.0) {
    return Result<std::vector<Line2D>>::Err(
        "DisplayLists line reader requires a physical sheet size");
  }
  const std::set<std::string> known(all_view_names.begin(),
                                    all_view_names.end());
  const std::set<std::string> selected(selected_view_names.begin(),
                                       selected_view_names.end());
  const std::set<uint32_t> selected_objects(selected_object_ids.begin(),
                                            selected_object_ids.end());
  const bool has_view_selection = !known.empty() && !selected.empty();
  const bool has_sheet_selection =
      !all_sheet_names.empty() && !selected_sheet_name.empty();
  if ((!has_view_selection && selected_objects.empty() &&
       !has_sheet_selection) ||
      display_lists.size() < kLinePayloadBytes) {
    return Result<std::vector<Line2D>>::Ok({});
  }
  const std::vector<ViewReference> references =
      FindViewReferences(display_lists, known);
  const std::vector<ObjectReference> object_references =
      FindObjectReferences(display_lists);
  const auto section_callouts =
      FindSectionCalloutOwnership(display_lists, object_references);
  const std::vector<ViewReference> sheet_end_references =
      FindSheetEndReferences(display_lists, all_sheet_names);
  if (references.empty() && object_references.empty() &&
      sheet_end_references.empty()) {
    return Result<std::vector<Line2D>>::Ok({});
  }

  std::vector<Line2D> lines;
  std::set<std::tuple<double, double, double, double>> seen;
  for (std::size_t offset = 0;
       offset + kLinePayloadBytes <= display_lists.size(); ++offset) {
    if (ReadU32Le(display_lists, offset) != kLineOpcode ||
        ReadU32Le(display_lists, offset + 4u) != kTwoPointPayload) {
      continue;
    }
    auto owner_label =
        SelectOwnerLabel(offset, object_references, selected_objects,
                         references, selected, section_callouts);
    if (!owner_label && has_sheet_selection) {
      const ViewReference *sheet_owner =
          FindSheetEndOwner(sheet_end_references, offset);
      if (sheet_owner != nullptr && sheet_owner->name == selected_sheet_name) {
        owner_label = "sheet=" + sheet_owner->name;
      }
    }
    if (!owner_label)
      continue;

    const Vec2 start = {ReadF64Le(display_lists, offset + 8u),
                        ReadF64Le(display_lists, offset + 16u)};
    const double start_z = ReadF64Le(display_lists, offset + 24u);
    const Vec2 end = {ReadF64Le(display_lists, offset + 32u),
                      ReadF64Le(display_lists, offset + 40u)};
    const double end_z = ReadF64Le(display_lists, offset + 48u);
    // X/Y are already sheet coordinates. A leader attached to an isometric
    // model point can retain a different Z at each endpoint.
    if (!IsSheetCoordinate(start.x, sheet_width_m) ||
        !IsSheetCoordinate(end.x, sheet_width_m) ||
        !IsSheetCoordinate(start.y, sheet_height_m) ||
        !IsSheetCoordinate(end.y, sheet_height_m) || !std::isfinite(start_z) ||
        !std::isfinite(end_z) || std::abs(start_z) > 1000.0 ||
        std::abs(end_z) > 1000.0) {
      continue;
    }
    const double length = std::hypot(end.x - start.x, end.y - start.y);
    if (!std::isfinite(length) || length <= 1e-10 ||
        length > std::hypot(sheet_width_m, sheet_height_m) + 1e-9) {
      continue;
    }
    const auto key = std::make_tuple(start.x, start.y, end.x, end.y);
    const auto reverse = std::make_tuple(end.x, end.y, start.x, start.y);
    if (seen.contains(key) || seen.contains(reverse))
      continue;
    seen.insert(key);
    const LineStyle style = ReadLineStyle(display_lists, offset);
    lines.push_back(Line2D{start, end, style,
                           "Contents/DisplayLists/line/" + *owner_label});
    offset += kLinePayloadBytes - 1u;
  }
  return Result<std::vector<Line2D>>::Ok(std::move(lines));
}

Result<std::vector<Arc2D>> DrawingGeometryReader::ReadDisplayListArcs(
    std::span<const uint8_t> display_lists,
    std::span<const std::string> all_view_names,
    std::span<const std::string> selected_view_names, double sheet_width_m,
    double sheet_height_m, std::span<const uint32_t> selected_object_ids,
    std::span<const std::string> all_sheet_names,
    std::string_view selected_sheet_name) {
  if (!std::isfinite(sheet_width_m) || !std::isfinite(sheet_height_m) ||
      sheet_width_m <= 0.0 || sheet_height_m <= 0.0) {
    return Result<std::vector<Arc2D>>::Err(
        "DisplayLists arc reader requires a physical sheet size");
  }
  const std::set<std::string> known(all_view_names.begin(),
                                    all_view_names.end());
  const std::set<std::string> selected(selected_view_names.begin(),
                                       selected_view_names.end());
  const std::set<uint32_t> selected_objects(selected_object_ids.begin(),
                                            selected_object_ids.end());
  const bool has_view_selection = !known.empty() && !selected.empty();
  const bool has_sheet_selection =
      !all_sheet_names.empty() && !selected_sheet_name.empty();
  if ((!has_view_selection && selected_objects.empty() &&
       !has_sheet_selection) ||
      display_lists.size() < kArcPayloadBytes) {
    return Result<std::vector<Arc2D>>::Ok({});
  }
  const std::vector<ViewReference> references =
      FindViewReferences(display_lists, known);
  const std::vector<ObjectReference> object_references =
      FindObjectReferences(display_lists);
  const auto section_callouts =
      FindSectionCalloutOwnership(display_lists, object_references);
  const std::vector<ViewReference> sheet_end_references =
      FindSheetEndReferences(display_lists, all_sheet_names);

  std::vector<Arc2D> arcs;
  std::set<std::tuple<double, double, double, double, double, double>> seen;
  for (std::size_t offset = 0;
       offset + kArcPayloadBytes <= display_lists.size(); ++offset) {
    if (ReadU32Le(display_lists, offset) != kArcOpcode ||
        ReadU32Le(display_lists, offset + 4u) != kArcPayload) {
      continue;
    }
    auto owner_label =
        SelectOwnerLabel(offset, object_references, selected_objects,
                         references, selected, section_callouts);
    if (!owner_label && has_sheet_selection) {
      const ViewReference *sheet_owner =
          FindSheetEndOwner(sheet_end_references, offset);
      if (sheet_owner != nullptr && sheet_owner->name == selected_sheet_name) {
        owner_label = "sheet=" + sheet_owner->name;
      }
    }
    if (!owner_label)
      continue;

    const Vec2 start = {ReadF64Le(display_lists, offset + 8u),
                        ReadF64Le(display_lists, offset + 16u)};
    const double start_z = ReadF64Le(display_lists, offset + 24u);
    const Vec2 end = {ReadF64Le(display_lists, offset + 32u),
                      ReadF64Le(display_lists, offset + 40u)};
    const double end_z = ReadF64Le(display_lists, offset + 48u);
    const Vec2 center = {ReadF64Le(display_lists, offset + 56u),
                         ReadF64Le(display_lists, offset + 64u)};
    const double center_z = ReadF64Le(display_lists, offset + 72u);
    const double normal_x = ReadF64Le(display_lists, offset + 80u);
    const double normal_y = ReadF64Le(display_lists, offset + 88u);
    const double normal_z = ReadF64Le(display_lists, offset + 96u);
    if (!IsSheetCoordinate(start.x, sheet_width_m) ||
        !IsSheetCoordinate(end.x, sheet_width_m) ||
        !IsSheetCoordinate(center.x, sheet_width_m) ||
        !IsSheetCoordinate(start.y, sheet_height_m) ||
        !IsSheetCoordinate(end.y, sheet_height_m) ||
        !IsSheetCoordinate(center.y, sheet_height_m) ||
        !std::isfinite(start_z) || !std::isfinite(end_z) ||
        !std::isfinite(center_z) || !std::isfinite(normal_x) ||
        !std::isfinite(normal_y) || !std::isfinite(normal_z) ||
        std::abs(start_z - end_z) > 1e-8 ||
        std::abs(start_z - center_z) > 1e-8 || std::abs(normal_x) > 1e-8 ||
        std::abs(normal_y) > 1e-8 ||
        std::abs(std::abs(normal_z) - 1.0) > 1e-8) {
      continue;
    }
    const double start_radius =
        std::hypot(start.x - center.x, start.y - center.y);
    const double end_radius = std::hypot(end.x - center.x, end.y - center.y);
    if (!std::isfinite(start_radius) || start_radius <= 1e-6 ||
        start_radius > std::hypot(sheet_width_m, sheet_height_m) ||
        std::abs(start_radius - end_radius) >
            std::max(1e-7, start_radius * 1e-5)) {
      continue;
    }
    const auto key =
        std::make_tuple(start.x, start.y, end.x, end.y, center.x, center.y);
    if (!seen.insert(key).second)
      continue;
    const LineStyle style = ReadLineStyle(display_lists, offset);
    arcs.push_back(Arc2D{start, end, center, normal_z, style,
                         "Contents/DisplayLists/arc/" + *owner_label});
    offset += kArcPayloadBytes - 1u;
  }
  return Result<std::vector<Arc2D>>::Ok(std::move(arcs));
}

Result<std::vector<TextAnnotation>> DrawingGeometryReader::ReadDisplayListText(
    std::span<const uint8_t> display_lists,
    std::span<const std::string> all_view_names,
    std::span<const std::string> selected_view_names, double sheet_width_m,
    double sheet_height_m, std::span<const uint32_t> selected_object_ids,
    std::span<const std::string> all_sheet_names,
    std::string_view selected_sheet_name) {
  if (!std::isfinite(sheet_width_m) || !std::isfinite(sheet_height_m) ||
      sheet_width_m <= 0.0 || sheet_height_m <= 0.0) {
    return Result<std::vector<TextAnnotation>>::Err(
        "DisplayLists text reader requires a physical sheet size");
  }
  const std::set<std::string> known(all_view_names.begin(),
                                    all_view_names.end());
  const std::set<std::string> selected(selected_view_names.begin(),
                                       selected_view_names.end());
  const std::set<uint32_t> selected_objects(selected_object_ids.begin(),
                                            selected_object_ids.end());
  const bool has_view_selection = !known.empty() && !selected.empty();
  const bool has_sheet_selection =
      !all_sheet_names.empty() && !selected_sheet_name.empty();
  if ((!has_view_selection && selected_objects.empty() &&
       !has_sheet_selection) ||
      display_lists.size() < kTextPrefixBytes) {
    return Result<std::vector<TextAnnotation>>::Ok({});
  }
  const std::vector<TextStyleReference> text_styles =
      FindTextStyles(display_lists);
  const std::vector<ViewReference> references =
      FindViewReferences(display_lists, known);
  const std::vector<ObjectReference> object_references =
      FindObjectReferences(display_lists);
  const auto section_callouts =
      FindSectionCalloutOwnership(display_lists, object_references);
  const std::vector<ViewReference> sheet_end_references =
      FindSheetEndReferences(display_lists, all_sheet_names);
  if (references.empty() && object_references.empty() &&
      sheet_end_references.empty()) {
    return Result<std::vector<TextAnnotation>>::Ok({});
  }

  std::vector<TextAnnotation> annotations;
  std::set<std::tuple<std::string, double, double, double, std::string>> seen;
  for (std::size_t offset = 0;
       offset + kTextPrefixBytes <= display_lists.size(); ++offset) {
    const uint32_t opcode = ReadU32Le(display_lists, offset);
    if (!IsTextOpcode(opcode) ||
        ReadU32Le(display_lists, offset + 4u) != kTextPayload) {
      continue;
    }
    auto owner_label =
        SelectOwnerLabel(offset, object_references, selected_objects,
                         references, selected, section_callouts);
    if (!owner_label && has_sheet_selection) {
      const ViewReference *sheet_owner =
          FindSheetEndOwner(sheet_end_references, offset);
      if (sheet_owner != nullptr && sheet_owner->name == selected_sheet_name) {
        owner_label = "sheet=" + sheet_owner->name;
      }
    }
    if (!owner_label)
      continue;

    const Vec2 origin = {ReadF64Le(display_lists, offset + 8u),
                         ReadF64Le(display_lists, offset + 16u)};
    const double z = ReadF64Le(display_lists, offset + 24u);
    double rotation = ReadF64Le(display_lists, offset + 36u);
    const auto value = ReadUtf16Field(display_lists, offset + 48u);
    if (!value || !IsSheetCoordinate(origin.x, sheet_width_m) ||
        !IsSheetCoordinate(origin.y, sheet_height_m) || !std::isfinite(z) ||
        std::abs(z) > 1000.0 || !std::isfinite(rotation) ||
        std::abs(rotation) > kTwoPi + 1e-6) {
      continue;
    }
    rotation = std::remainder(rotation, kTwoPi);
    if (std::abs(rotation) < 1e-10)
      rotation = 0.0;
    if (!seen.emplace(*value, origin.x, origin.y, rotation, *owner_label)
             .second)
      continue;

    const auto *style = FindTextStyle(text_styles, offset);
    const double height =
        style ? style->height_m : kDefaultDimensionTextHeightM;
    TextAnnotation annotation{*value,
                              origin,
                              rotation,
                              height,
                              0.0,
                              style ? style->font_family : "Century Gothic",
                              "Contents/DisplayLists/text/" +
                                  std::to_string(opcode) + "/" + *owner_label,
                              TextAnchor::kStart};
    annotation.rendered_width =
        ReadTextRenderedWidth(display_lists, offset + 48u).value_or(0.0);
    annotation.bold = style && style->bold;
    annotations.push_back(std::move(annotation));
  }
  return Result<std::vector<TextAnnotation>>::Ok(std::move(annotations));
}

Result<std::vector<uint32_t>> DrawingGeometryReader::ReadDisplayObjectIds(
    std::span<const uint8_t> definition,
    std::span<const std::string> sheet_names, std::string_view sheet_name) {
  const std::set<std::string> known_sheets(sheet_names.begin(),
                                           sheet_names.end());
  if (!known_sheets.contains(std::string(sheet_name))) {
    return Result<std::vector<uint32_t>>::Err(
        "Requested sheet is not present in the supplied sheet names");
  }
  const std::vector<ViewReference> sheet_references =
      FindExactReferences(definition, known_sheets);
  if (sheet_references.empty()) {
    return Result<std::vector<uint32_t>>::Ok({});
  }

  std::set<uint32_t> ids;
  constexpr std::array<uint8_t, 8> kDefinitionObjectSuffix = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x4A};
  constexpr std::array<uint8_t, 8> kLegacyNoteObjectSuffix = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x46};
  for (std::size_t offset = 0; offset + 12u <= definition.size(); ++offset) {
    if (!std::equal(
            kDefinitionObjectSuffix.begin(), kDefinitionObjectSuffix.end(),
            definition.begin() + static_cast<std::ptrdiff_t>(offset + 4u)) &&
        !std::equal(
            kLegacyNoteObjectSuffix.begin(), kLegacyNoteObjectSuffix.end(),
            definition.begin() + static_cast<std::ptrdiff_t>(offset + 4u))) {
      continue;
    }
    const uint32_t id = ReadU32Le(definition, offset);
    if (id == 0u || id > 10000000u)
      continue;
    const auto owner = std::upper_bound(
        sheet_references.begin(), sheet_references.end(), offset,
        [](std::size_t value, const ViewReference &reference) {
          return value < reference.offset;
        });
    if (owner != sheet_references.begin() &&
        std::prev(owner)->name == sheet_name) {
      ids.insert(id);
    }
  }
  return Result<std::vector<uint32_t>>::Ok(
      std::vector<uint32_t>(ids.begin(), ids.end()));
}

} // namespace openswx
