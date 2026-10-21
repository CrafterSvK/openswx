// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "delta_packets.h"

#include <algorithm>
#include <array>

#include "binary_reader.h"
#include "openparasolid/schema.h"

namespace openparasolid {
namespace {
using internal::Cursor;
using internal::Fail;
constexpr std::array<uint8_t, 9> kHistoryTerminalPrefix{0, 1, 0, 1, 0,
                                                        1, 0, 0, 0};
bool Matches(std::span<const uint8_t> data, uint64_t offset,
             std::span<const uint8_t> expected) {
  return offset <= data.size() && expected.size() <= data.size() - offset &&
         std::equal(expected.begin(), expected.end(),
                    data.begin() + static_cast<std::ptrdiff_t>(offset));
}
DeltaPacket Map(std::span<const uint8_t> data, uint64_t offset,
                uint16_t maximum, uint64_t limit) {
  Cursor cursor{data, offset + 4};
  DeltaPacket result;
  result.offset = offset;
  while (true) {
    const auto start = cursor.pos;
    const auto index = cursor.Index();
    if (index == 0) {
      if (cursor.Read(2) != 0)
        Fail(
            ErrorCode::kUnsupportedFormat, start,
            "Reference map requires a self-delimiting null/zero/word terminal");
      const auto word = static_cast<uint16_t>(cursor.Read(2));
      if (word == 0)
        Fail(ErrorCode::kUnsupportedFormat, cursor.pos - 2,
             "Reference-map terminal form with a zero word is not supported");
      result.has_terminal = true;
      result.terminal_word = word;
      result.size = cursor.pos - offset;
      return result;
    }
    if (result.references.size() >= limit)
      Fail(ErrorCode::kResourceLimit, start,
           "Delta packet entry limit exceeded");
    const auto type = static_cast<uint16_t>(cursor.Read(2));
    if (type < 2 || type > 32767 || type > maximum)
      Fail(ErrorCode::kInvalidRecord, cursor.pos - 2,
           "Reference-map node type outside the schema range");
    result.references.push_back({start, cursor.pos - start, index, type});
  }
}
DeltaPacket History(std::span<const uint8_t> data, uint64_t offset,
                    uint64_t limit) {
  Cursor cursor{data, offset + 4};
  DeltaPacket result;
  result.kind = DeltaPacketKind::kHistoricalMarks;
  result.offset = offset;
  do {
    const auto start = cursor.pos;
    if (cursor.Read(2) != 4)
      Fail(ErrorCode::kInvalidRecord, start,
           "Historical packet requires a PMARK node");
    if (result.marks.size() >= limit)
      Fail(ErrorCode::kResourceLimit, start,
           "Delta packet entry limit exceeded");
    DeltaMark mark;
    mark.offset = start;
    mark.index = cursor.Index();
    if (!mark.index)
      Fail(ErrorCode::kInvalidRecord, cursor.pos,
           "Historical PMARK index must be non-null");
    mark.preceding = cursor.Index();
    mark.first_following = cursor.Index();
    mark.next_sibling = cursor.Index();
    mark.prev_sibling = cursor.Index();
    mark.n_new_nodes = static_cast<uint32_t>(cursor.Read(4));
    mark.n_del_nodes = static_cast<uint32_t>(cursor.Read(4));
    mark.n_copy_mod_nodes = static_cast<uint32_t>(cursor.Read(4));
    mark.delta_key = static_cast<uint32_t>(cursor.Read(4));
    mark.delta_is_forward = cursor.Logical();
    mark.identifier = static_cast<uint32_t>(cursor.Read(4));
    result.marks.push_back({mark, cursor.pos - start});
  } while (Matches(data, cursor.pos, std::array<uint8_t, 2>{0, 4}));
  // Both observed forms belong to the preceding historical packet. Keep
  // the final word opaque; it is not a map entry or a direction flag.
  if (Matches(data, cursor.pos, kHistoryTerminalPrefix) &&
      data.size() - cursor.pos >= 10 && data[cursor.pos + 9] <= 1) {
    result.terminal_word = data[cursor.pos + 9];
    cursor.pos += 10;
    result.has_terminal = true;
  }
  result.size = cursor.pos - offset;
  return result;
}
enum class Boundary { kUnavailable, kInRange, kInvalid };
Boundary FollowingType(std::span<const uint8_t> data, const DeltaPacket& packet,
                       uint16_t maximum) {
  Cursor cursor{data, packet.offset + packet.size};
  if (data.size() - cursor.pos < 2) return Boundary::kUnavailable;
  const auto type = cursor.Read(2);
  return type >= 1 && type <= maximum ? Boundary::kInRange : Boundary::kInvalid;
}
Result<DeltaPacket> ReadPacket(std::span<const uint8_t> data, uint64_t offset,
                               uint16_t maximum, uint64_t limit,
                               bool sequential) {
  try {
    if (!limit || maximum < 4 || maximum > 32767)
      Fail(ErrorCode::kInvalidOptions, offset, "Invalid delta packet limits");
    Cursor prefix{data, offset};
    if (prefix.Read(4) != 0x00010001)
      Fail(ErrorCode::kUnsupportedFormat, offset,
           "Unsupported delta packet prefix");
    std::optional<DeltaPacket> map, history;
    std::optional<ParseError> map_error, history_error;
    try {
      map = Map(data, offset, maximum, limit);
    } catch (const ParseError& error) {
      map_error = error;
    }
    if (Matches(data, offset + 4, std::array<uint8_t, 2>{0, 4})) {
      try {
        history = History(data, offset, limit);
      } catch (const ParseError& error) {
        history_error = error;
      }
    }
    // A resource cap must not select the other interpretation by elimination.
    for (const auto* error : {&map_error, &history_error})
      if (*error && (*error)->code == ErrorCode::kResourceLimit) throw **error;
    if (map && history) {
      if (sequential) {
        const auto map_boundary = FollowingType(data, *map, maximum);
        const auto history_boundary = FollowingType(data, *history, maximum);
        if (map_boundary == Boundary::kInvalid &&
            history_boundary == Boundary::kInRange)
          return Result<DeltaPacket>::Ok(std::move(*history));
        if (history_boundary == Boundary::kInvalid &&
            map_boundary == Boundary::kInRange)
          return Result<DeltaPacket>::Ok(std::move(*map));
        if (map_boundary == Boundary::kInvalid &&
            history_boundary == Boundary::kInvalid)
          Fail(ErrorCode::kInvalidRecord, offset,
               "Neither delta packet interpretation ends before an in-range "
               "node type");
      }
      Fail(ErrorCode::kUnsupportedFormat, offset,
           "Ambiguous reference-map/historical-PMARK packet");
    }
    if (history) return Result<DeltaPacket>::Ok(std::move(*history));
    if (map) return Result<DeltaPacket>::Ok(std::move(*map));
    if (history_error) throw *history_error;
    throw *map_error;
  } catch (const ParseError& error) {
    return Result<DeltaPacket>::Err(error);
  }
}
}  // namespace
const char* ToString(DeltaPacketKind kind) noexcept {
  switch (kind) {
    case DeltaPacketKind::kReferenceMap:
      return "reference_map";
    case DeltaPacketKind::kHistoricalMarks:
      return "historical_marks";
  }
  return "unknown";
}
Result<DeltaPacket> DeltaPacketReader::Read(std::span<const uint8_t> data,
                                            uint64_t offset, uint16_t maximum,
                                            uint64_t limit) {
  return ReadPacket(data, offset, maximum, limit, false);
}
namespace internal {
Result<DeltaPacket> ReadSequentialDeltaPacket(std::span<const uint8_t> data,
                                              uint64_t offset, uint16_t maximum,
                                              uint64_t limit) {
  return ReadPacket(data, offset, maximum, limit, true);
}
}  // namespace internal
}  // namespace openparasolid
