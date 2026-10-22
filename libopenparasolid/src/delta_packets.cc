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
constexpr std::array<uint8_t, 9> kHistoryTerminalPrefix{0, 1, 0, 1, 0,
                                                        1, 0, 0, 0};
bool Matches(std::span<const uint8_t> data, uint64_t offset,
             std::span<const uint8_t> expected) {
  return offset <= data.size() && expected.size() <= data.size() - offset &&
         std::equal(expected.begin(), expected.end(),
                    data.begin() + static_cast<std::ptrdiff_t>(offset));
}
Result<DeltaPacket> Map(std::span<const uint8_t> data, uint64_t offset,
                        uint16_t maximum, uint64_t limit) {
  using Return = Result<DeltaPacket>;
  Cursor cursor{data, offset + 4};
  DeltaPacket result;
  result.offset = offset;
  while (true) {
    const auto start = cursor.pos;
    const auto index_result = cursor.Index();
    if (!index_result.ok()) return Return::Err(index_result.error());
    const auto index = index_result.value();
    if (index == 0) {
      const auto zero_result = cursor.Read(2);
      if (!zero_result.ok()) return Return::Err(zero_result.error());
      if (zero_result.value() != 0)
        return Return::Err({ErrorCode::kUnsupportedFormat, start,
                            "Reference map requires a self-delimiting "
                            "null/zero/word terminal"});
      const auto word_result = cursor.Read(2);
      if (!word_result.ok()) return Return::Err(word_result.error());
      const auto word = static_cast<uint16_t>(word_result.value());
      if (word == 0)
        return Return::Err(
            {ErrorCode::kUnsupportedFormat, cursor.pos - 2,
             "Reference-map terminal form with a zero word is not supported"});
      result.has_terminal = true;
      result.terminal_word = word;
      result.size = cursor.pos - offset;
      return Return::Ok(std::move(result));
    }
    if (result.references.size() >= limit)
      return Return::Err({ErrorCode::kResourceLimit, start,
                          "Delta packet entry limit exceeded"});
    const auto type_result = cursor.Read(2);
    if (!type_result.ok()) return Return::Err(type_result.error());
    const auto type = static_cast<uint16_t>(type_result.value());
    if (type < 2 || type > 32767 || type > maximum)
      return Return::Err({ErrorCode::kInvalidRecord, cursor.pos - 2,
                          "Reference-map node type outside the schema range"});
    result.references.push_back({start, cursor.pos - start, index, type});
  }
}
Result<DeltaPacket> History(std::span<const uint8_t> data, uint64_t offset,
                            uint64_t limit) {
  using Return = Result<DeltaPacket>;
  Cursor cursor{data, offset + 4};
  DeltaPacket result;
  result.kind = DeltaPacketKind::kHistoricalMarks;
  result.offset = offset;
  do {
    const auto start = cursor.pos;
    const auto type_result = cursor.Read(2);
    if (!type_result.ok()) return Return::Err(type_result.error());
    if (type_result.value() != 4)
      return Return::Err({ErrorCode::kInvalidRecord, start,
                          "Historical packet requires a PMARK node"});
    if (result.marks.size() >= limit)
      return Return::Err({ErrorCode::kResourceLimit, start,
                          "Delta packet entry limit exceeded"});
    DeltaMark mark;
    mark.offset = start;
    const auto index_result = cursor.Index();
    if (!index_result.ok()) return Return::Err(index_result.error());
    mark.index = index_result.value();
    if (!mark.index)
      return Return::Err({ErrorCode::kInvalidRecord, cursor.pos,
                          "Historical PMARK index must be non-null"});
    const auto preceding_result = cursor.Index();
    if (!preceding_result.ok()) return Return::Err(preceding_result.error());
    mark.preceding = preceding_result.value();
    const auto following_result = cursor.Index();
    if (!following_result.ok()) return Return::Err(following_result.error());
    mark.first_following = following_result.value();
    const auto next_result = cursor.Index();
    if (!next_result.ok()) return Return::Err(next_result.error());
    mark.next_sibling = next_result.value();
    const auto previous_result = cursor.Index();
    if (!previous_result.ok()) return Return::Err(previous_result.error());
    mark.prev_sibling = previous_result.value();
    const auto new_count = cursor.Read(4);
    if (!new_count.ok()) return Return::Err(new_count.error());
    mark.n_new_nodes = static_cast<uint32_t>(new_count.value());
    const auto deleted_count = cursor.Read(4);
    if (!deleted_count.ok()) return Return::Err(deleted_count.error());
    mark.n_del_nodes = static_cast<uint32_t>(deleted_count.value());
    const auto modified_count = cursor.Read(4);
    if (!modified_count.ok()) return Return::Err(modified_count.error());
    mark.n_copy_mod_nodes = static_cast<uint32_t>(modified_count.value());
    const auto key_result = cursor.Read(4);
    if (!key_result.ok()) return Return::Err(key_result.error());
    mark.delta_key = static_cast<uint32_t>(key_result.value());
    const auto direction_result = cursor.Logical();
    if (!direction_result.ok()) return Return::Err(direction_result.error());
    mark.delta_is_forward = direction_result.value();
    const auto identifier_result = cursor.Read(4);
    if (!identifier_result.ok()) return Return::Err(identifier_result.error());
    mark.identifier = static_cast<uint32_t>(identifier_result.value());
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
  return Return::Ok(std::move(result));
}
enum class Boundary { kUnavailable, kInRange, kInvalid };
Boundary FollowingType(std::span<const uint8_t> data, const DeltaPacket& packet,
                       uint16_t maximum) {
  Cursor cursor{data, packet.offset + packet.size};
  const auto type = cursor.Read(2);
  if (!type.ok()) return Boundary::kUnavailable;
  return type.value() >= 1 && type.value() <= maximum ? Boundary::kInRange
                                                      : Boundary::kInvalid;
}
Result<DeltaPacket> ReadPacket(std::span<const uint8_t> data, uint64_t offset,
                               uint16_t maximum, uint64_t limit,
                               bool sequential) {
  using Return = Result<DeltaPacket>;
  if (!limit || maximum < 4 || maximum > 32767)
    return Return::Err(
        {ErrorCode::kInvalidOptions, offset, "Invalid delta packet limits"});
  Cursor prefix{data, offset};
  const auto signature = prefix.Read(4);
  if (!signature.ok()) return Return::Err(signature.error());
  if (signature.value() != 0x00010001)
    return Return::Err({ErrorCode::kUnsupportedFormat, offset,
                        "Unsupported delta packet prefix"});
  auto map = Map(data, offset, maximum, limit);
  std::optional<Result<DeltaPacket>> history;
  if (Matches(data, offset + 4, std::array<uint8_t, 2>{0, 4}))
    history = History(data, offset, limit);
  // A resource cap must not select the other interpretation by elimination.
  if (!map.ok() && map.error().code == ErrorCode::kResourceLimit)
    return Return::Err(map.error());
  if (history && !history->ok() &&
      history->error().code == ErrorCode::kResourceLimit)
    return Return::Err(history->error());
  if (map.ok() && history && history->ok()) {
    if (sequential) {
      const auto map_boundary = FollowingType(data, map.value(), maximum);
      const auto history_boundary =
          FollowingType(data, history->value(), maximum);
      if (map_boundary == Boundary::kInvalid &&
          history_boundary == Boundary::kInRange)
        return std::move(*history);
      if (history_boundary == Boundary::kInvalid &&
          map_boundary == Boundary::kInRange)
        return map;
      if (map_boundary == Boundary::kInvalid &&
          history_boundary == Boundary::kInvalid)
        return Return::Err(
            {ErrorCode::kInvalidRecord, offset,
             "Neither delta packet interpretation ends before an in-range "
             "node type"});
    }
    return Return::Err({ErrorCode::kUnsupportedFormat, offset,
                        "Ambiguous reference-map/historical-PMARK packet"});
  }
  if (history && history->ok()) return std::move(*history);
  if (map.ok()) return map;
  if (history) return Return::Err(history->error());
  return Return::Err(map.error());
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
