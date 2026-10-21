// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "../tools/inspect_options.h"

#include <gtest/gtest.h>

namespace openparasolid::cli {
namespace {
Result<InspectSettings> Parse(
    std::initializer_list<std::string_view> args,
    const std::map<std::string, std::string>& env = {}) {
  return ParseInspectSettings(
      std::span(args.begin(), args.size()),
      [&env](std::string_view name) -> std::optional<std::string> {
        const auto found = env.find(std::string(name));
        return found == env.end() ? std::nullopt
                                  : std::optional<std::string>(found->second);
      });
}
TEST(ParasolidInspectOptions, EnvironmentThenCommandLinePrecedence) {
  const auto result =
      Parse({"--geometry=false", "--nodes", "--max-records", "7", "part.x_b"},
            {{"OPENPARASOLID_GEOMETRY", "true"},
             {"OPENPARASOLID_MAX_RECORDS", "invalid"},
             {"OPENPARASOLID_MAX_FIELDS", "1000"},
             {"OPENPARASOLID_SUMMARY", "1"}});
  ASSERT_TRUE(result.ok());
  EXPECT_FALSE(result.value().geometry);
  EXPECT_TRUE(result.value().nodes);
  EXPECT_TRUE(result.value().summary);
  EXPECT_EQ(result.value().limits.max_nodes, 7);
  EXPECT_EQ(result.value().limits.max_total_fields, 1000);
}
TEST(ParasolidInspectOptions, GeometryAndCatalogEachEnableSequentialMode) {
  auto result = Parse({"part.x_b"}, {{"OPENPARASOLID_GEOMETRY", "1"}});
  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(result.value().geometry);
  EXPECT_TRUE(result.value().nodes);
  result =
      Parse({"part.x_b"}, {{"OPENPARASOLID_SCHEMA_CATALOG", "schema.json"}});
  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(result.value().nodes);
  ASSERT_TRUE(result.value().catalog.has_value());
  EXPECT_EQ(*result.value().catalog, "schema.json");
}
TEST(ParasolidInspectOptions,
     RejectsInvalidEnvironmentAndDoesNotSilentlySwitchModes) {
  EXPECT_FALSE(Parse({"part.x_b"}, {{"OPENPARASOLID_GEOMETRY", "yes"}}).ok());
  EXPECT_FALSE(
      Parse({"--geometry", "part.x_b"}, {{"OPENPARASOLID_MAX_VALUES", "0"}})
          .ok());
  EXPECT_FALSE(Parse({"--nodes", "part.x_b"},
                     {{"OPENPARASOLID_MAX_FIELDS", "18446744073709551616"}})
                   .ok());
  EXPECT_FALSE(Parse({"part.x_b"}, {{"OPENPARASOLID_MAX_FIELDS", "10"}}).ok());
  EXPECT_FALSE(Parse({"--geometry", "part.x_b", "other.x_b"}).ok());
  EXPECT_FALSE(Parse({"--geometry"}).ok());
}
TEST(ParasolidInspectOptions, DefaultsAndExplicitBooleanOverridesAreStable) {
  const auto result = Parse(
      {"--summary=false", "--nodes=false", "part.x_b"},
      {{"OPENPARASOLID_SUMMARY", "true"}, {"OPENPARASOLID_NODES", "true"}});
  ASSERT_TRUE(result.ok());
  EXPECT_FALSE(result.value().summary);
  EXPECT_FALSE(result.value().nodes);
  EXPECT_EQ(result.value().limits.max_nodes, 10000);
  EXPECT_EQ(result.value().limits.max_values, 1000000);
  EXPECT_EQ(result.value().limits.max_total_fields, 65536);
}
}  // namespace
}  // namespace openparasolid::cli
