// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

// internal/modern_parser.h — Parser for the SolidWorks 2015+ binary format.
//
// Format overview
// ───────────────
// SolidWorks files exist in three distinct container formats:
//
//   1. OLE2 / CFB  (legacy, pre-2015)
//        Magic: D0 CF 11 E0 A1 B1 1A E1
//        Detected by: IsOle2()  (see ole2_parser.h)
//
//   2. SW chunk format  (modern, 2015+)
//        Magic: varies per file (4 bytes at offset 0, file-specific)
//        ROL key: 1 byte at offset 7 — decodes all stream names
//        First chunk marker: 14 00 06 00 08 00  within the first 64 bytes
//        Detected by: IsModernSwx()
//
//   3. ZIP / OPC  (3DExperience, future)
//        Magic: 50 4B 03 04  ("PK\x03\x04")
//        Detected by: IsZipOpc()
//        Not yet parsed — reserved for future support.
//
// Detection order (use in this sequence):
//   if      IsOle2(data)      → ParseOle2Format()
//   else if IsZipOpc(data)    → (future ZIP/OPC parser)
//   else if IsModernSwx(data) → ParseModernFormat()
//   else                      → unrecognised format
//
// Chunk format details
// ────────────────────
// Chunks are found by scanning for the 6-byte marker 14 00 06 00 08 00.
// Inline blocks use this layout relative to the marker:
//
//   +0x00  14 00 06 00 08 00  (6-byte marker)
//   +0x06  type_id  uint32 LE
//   +0x0a  CRC32    uint32 LE — checksum of uncompressed payload
//   +0x0e  csz      uint32 LE — compressed size
//   +0x12  usz      uint32 LE — uncompressed size
//   +0x16  nsz      uint32 LE — stream name length (bytes)
//   +0x1a  name[nsz]          — ROL-decoded UTF-8 stream name
//   +0x1a+nsz  data[csz]      — raw-deflate payload
//
// Version stream
// ──────────────
// The file version is encoded in a stream *name* with the prefix
// "_MO_VERSION_NNNNN/" (decimal).  There is no separate content stream.
// Extract via ExtractVersion() in document.cc.

#include <cstdint>
#include <span>

#include "internal/stream_store.h"
#include "openswx/container.h"

namespace openswx::internal {

// Returns true iff data begins with the ZIP/OPC magic 50 4B 03 04.
// Used to detect 3DExperience / Open Packaging Convention files.
[[nodiscard]] bool IsZipOpc(std::span<const uint8_t> data) noexcept;

// Returns true iff data looks like a modern SolidWorks chunk-format file:
//   - does NOT start with OLE2 or ZIP/OPC magic
//   - the 6-byte chunk marker 14 00 06 00 08 00 appears within the first
//     64 bytes (empirically always at offset 13–21 in known SW files)
[[nodiscard]] bool IsModernSwx(std::span<const uint8_t> data) noexcept;

// Parses a modern-format SolidWorks file and populates streams.
// Streams are added; any pre-existing entries are not overwritten.
// Returns false only on a hard parse error (file too short).
// Unrecognised or malformed chunks are skipped.  When inspection is supplied,
// inline streams, cache cells, directory entries, unknown markers, and
// unclaimed byte ranges are retained for diagnostics.
[[nodiscard]] bool ParseModernFormat(std::span<const uint8_t> file_data,
                                     StreamMap& streams,
                                     ContainerInspection* inspection = nullptr);

// Returns true iff name consists only of printable ASCII [0x20, 0x7f).
// Used to reject garbled ROL-decoded names from misidentified data.
[[nodiscard]] bool IsValidStreamName(const std::string& name) noexcept;

}  // namespace openswx::internal
