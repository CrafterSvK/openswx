// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace openswx::internal {

// Extracts absolute Windows paths ending in .SLDPRT or .SLDASM from binary
// streams that store their reference history as UTF-16LE strings.
[[nodiscard]] std::vector<std::string> ExtractUtf16CadReferences(
    std::span<const uint8_t> data);

}  // namespace openswx::internal
