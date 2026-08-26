// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "internal/reference_reader.h"

#include <algorithm>
#include <cctype>
#include <set>
#include <string>
#include <string_view>

namespace openswx::internal {
namespace {

bool IsPathStart(std::span<const uint8_t> data, std::size_t offset) {
  if (offset + 6 > data.size()) return false;
  const bool drive_path =
      std::isalpha(data[offset]) && data[offset + 1] == 0 &&
      data[offset + 2] == ':' && data[offset + 3] == 0 &&
      data[offset + 4] == '\\' && data[offset + 5] == 0;
  const bool unc_path =
      data[offset] == '\\' && data[offset + 1] == 0 &&
      data[offset + 2] == '\\' && data[offset + 3] == 0;
  return drive_path || unc_path;
}

void AppendUtf8(std::string& output, uint16_t code_point) {
  if (code_point < 0x80) {
    output += static_cast<char>(code_point);
  } else if (code_point < 0x800) {
    output += static_cast<char>(0xC0 | (code_point >> 6));
    output += static_cast<char>(0x80 | (code_point & 0x3F));
  } else if (code_point < 0xD800 || code_point > 0xDFFF) {
    output += static_cast<char>(0xE0 | (code_point >> 12));
    output += static_cast<char>(0x80 | ((code_point >> 6) & 0x3F));
    output += static_cast<char>(0x80 | (code_point & 0x3F));
  }
}

bool HasCadExtension(const std::string& path) {
  constexpr std::string_view extensions[] = {".sldprt", ".sldasm"};
  for (const auto extension : extensions) {
    if (path.size() < extension.size()) continue;
    const auto suffix = path.substr(path.size() - extension.size());
    if (std::equal(suffix.begin(), suffix.end(), extension.begin(),
                   [](unsigned char left, unsigned char right) {
                     return std::tolower(left) == std::tolower(right);
                   })) {
      return true;
    }
  }
  return false;
}

std::string CaseFoldAscii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  return value;
}

}  // namespace

std::vector<std::string> ExtractUtf16CadReferences(
    std::span<const uint8_t> data) {
  std::vector<std::string> references;
  std::set<std::string> seen;

  for (std::size_t offset = 0; offset + 6 <= data.size(); ++offset) {
    if (!IsPathStart(data, offset)) continue;

    std::string path;
    for (std::size_t cursor = offset; cursor + 1 < data.size(); cursor += 2) {
      const uint16_t code_point =
          static_cast<uint16_t>(data[cursor]) |
          (static_cast<uint16_t>(data[cursor + 1]) << 8);
      if (code_point < 0x20 || code_point == 0xFFFF) break;
      AppendUtf8(path, code_point);
      if (!HasCadExtension(path)) continue;

      if (seen.insert(CaseFoldAscii(path)).second)
        references.push_back(std::move(path));
      offset = cursor + 1;
      break;
    }
  }

  return references;
}

}  // namespace openswx::internal
