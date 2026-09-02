// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <zlib.h>

#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace openswx::test {

inline void AppendU32Le(std::vector<uint8_t>* out, uint32_t value) {
  out->push_back(static_cast<uint8_t>(value & 0xFFu));
  out->push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
  out->push_back(static_cast<uint8_t>((value >> 16) & 0xFFu));
  out->push_back(static_cast<uint8_t>((value >> 24) & 0xFFu));
}

inline std::vector<uint8_t> DeflateRaw(std::span<const uint8_t> input) {
  z_stream stream{};
  stream.next_in = const_cast<Bytef*>(
      reinterpret_cast<const Bytef*>(input.data()));
  stream.avail_in = static_cast<uInt>(input.size());

  if (deflateInit2(&stream, Z_BEST_SPEED, Z_DEFLATED, -15, 8,
                   Z_DEFAULT_STRATEGY) != Z_OK) {
    throw std::runtime_error("deflateInit2 failed");
  }

  std::vector<uint8_t> output(64);
  int ret = Z_OK;
  while (ret == Z_OK) {
    if (stream.total_out == output.size()) {
      output.resize(output.size() * 2);
    }

    stream.next_out = reinterpret_cast<Bytef*>(output.data() + stream.total_out);
    stream.avail_out = static_cast<uInt>(output.size() - stream.total_out);
    ret = deflate(&stream, Z_FINISH);
  }

  deflateEnd(&stream);
  if (ret != Z_STREAM_END) {
    throw std::runtime_error("deflate failed");
  }

  output.resize(stream.total_out);
  return output;
}

inline std::vector<uint8_t> DeflateZlib(std::span<const uint8_t> input) {
  uLongf output_size = compressBound(static_cast<uLong>(input.size()));
  std::vector<uint8_t> output(output_size);
  if (compress2(output.data(), &output_size, input.data(),
                static_cast<uLong>(input.size()), Z_BEST_SPEED) != Z_OK) {
    throw std::runtime_error("compress2 failed");
  }
  output.resize(output_size);
  return output;
}

inline std::vector<uint8_t> MakeZlbBlock(std::span<const uint8_t> payload) {
  const std::vector<uint8_t> compressed = DeflateRaw(payload);
  std::vector<uint8_t> block(24, 0);
  block[16] = static_cast<uint8_t>(payload.size() & 0xFFu);
  block[17] = static_cast<uint8_t>((payload.size() >> 8) & 0xFFu);
  block[18] = static_cast<uint8_t>((payload.size() >> 16) & 0xFFu);
  block[19] = static_cast<uint8_t>((payload.size() >> 24) & 0xFFu);
  block[20] = static_cast<uint8_t>(compressed.size() & 0xFFu);
  block[21] = static_cast<uint8_t>((compressed.size() >> 8) & 0xFFu);
  block[22] = static_cast<uint8_t>((compressed.size() >> 16) & 0xFFu);
  block[23] = static_cast<uint8_t>((compressed.size() >> 24) & 0xFFu);
  block.insert(block.end(), compressed.begin(), compressed.end());
  return block;
}

inline std::vector<uint8_t> MakeModernChunkFile(
    const std::string& stream_name, std::span<const uint8_t> stream_payload,
    uint32_t type_id = 0xFD) {
  const std::vector<uint8_t> compressed = DeflateRaw(stream_payload);

  std::vector<uint8_t> file(8, 0);
  file.push_back(0xAA);
  file.push_back(0xBB);
  file.push_back(0xCC);
  file.push_back(0xDD);
  file.push_back(0x14);
  file.push_back(0x00);
  file.push_back(0x06);
  file.push_back(0x00);
  file.push_back(0x08);
  file.push_back(0x00);
  AppendU32Le(&file, type_id);

  const uLong payload_crc = crc32(
      crc32(0L, Z_NULL, 0), stream_payload.data(),
      static_cast<uInt>(stream_payload.size()));
  AppendU32Le(&file, static_cast<uint32_t>(payload_crc));
  AppendU32Le(&file, static_cast<uint32_t>(compressed.size()));
  AppendU32Le(&file, static_cast<uint32_t>(stream_payload.size()));
  AppendU32Le(&file, static_cast<uint32_t>(stream_name.size()));
  file.insert(file.end(), stream_name.begin(), stream_name.end());
  file.insert(file.end(), compressed.begin(), compressed.end());
  return file;
}

inline void AppendModernMarker(std::vector<uint8_t>* out) {
  out->insert(out->end(), {0x14, 0x00, 0x06, 0x00, 0x08, 0x00});
}

inline std::vector<uint8_t> MakeCacheCellFile(
    const std::string& stream_name, uint32_t logical_size,
    uint32_t type_id = 0xFD) {
  std::vector<uint8_t> file(8, 0);
  AppendModernMarker(&file);
  AppendU32Le(&file, type_id);
  AppendU32Le(&file, logical_size * 2u);
  AppendU32Le(&file, logical_size / 2u);
  AppendU32Le(&file, logical_size);
  AppendU32Le(&file, static_cast<uint32_t>(stream_name.size()));
  file.insert(file.end(), stream_name.begin(), stream_name.end());
  return file;
}

inline std::vector<uint8_t> MakeDirectoryEntryFile(
    const std::string& stream_name, uint32_t stored_size,
    uint32_t type_id = 0xDF) {
  std::vector<uint8_t> file(8, 0);
  AppendModernMarker(&file);
  AppendU32Le(&file, type_id);
  AppendU32Le(&file, 0);
  AppendU32Le(&file, stored_size);
  AppendU32Le(&file, 0);
  AppendU32Le(&file, static_cast<uint32_t>(stream_name.size()));
  file.insert(file.end(), 14, 0xA5);
  file.insert(file.end(), stream_name.begin(), stream_name.end());
  file.insert(file.end(), 6, 0);
  return file;
}

}  // namespace openswx::test
