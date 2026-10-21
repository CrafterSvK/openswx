// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#pragma once

#include "openparasolid/schema.h"

namespace openparasolid::internal {
// The input is a complete sequential delta stream, not a packet prefix.
Result<DeltaPacket> ReadSequentialDeltaPacket(std::span<const uint8_t> data,
                                              uint64_t offset, uint16_t maximum,
                                              uint64_t limit);
}  // namespace openparasolid::internal
