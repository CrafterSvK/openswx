// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include <gtest/gtest.h>

#include <cstdint>
#include <string_view>
#include <vector>

#include "internal/reference_reader.h"

namespace openswx::internal {
namespace {

void AppendUtf16Le(std::vector<uint8_t>* data, std::string_view value) {
  for (const unsigned char c : value) {
    data->push_back(c);
    data->push_back(0);
  }
  data->push_back(0xFF);
  data->push_back(0xFF);
}

TEST(ReferenceReaderTest, ExtractsAndDeduplicatesCadPaths) {
  std::vector<uint8_t> data = {0x03, 0x00, 0xFF, 0xFE};
  AppendUtf16Le(&data, "C:\\old\\Part.SLDPRT");
  AppendUtf16Le(&data, "C:\\new\\Assembly.SLDASM");
  AppendUtf16Le(&data, "c:\\OLD\\part.sldprt");
  AppendUtf16Le(&data, "C:\\ignored\\notes.txt");

  EXPECT_EQ(ExtractUtf16CadReferences(data),
            (std::vector<std::string>{"C:\\old\\Part.SLDPRT",
                                      "C:\\new\\Assembly.SLDASM"}));
}

TEST(ReferenceReaderTest, SupportsUncPaths) {
  std::vector<uint8_t> data;
  AppendUtf16Le(&data, "\\\\server\\models\\Part.SLDPRT");

  EXPECT_EQ(ExtractUtf16CadReferences(data),
            (std::vector<std::string>{
                "\\\\server\\models\\Part.SLDPRT"}));
}

}  // namespace
}  // namespace openswx::internal
