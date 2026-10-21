// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "internal/xml_readers.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace openswx::internal {
namespace {

std::vector<uint8_t> ToBytes(const std::string& text) {
  return std::vector<uint8_t>(text.begin(), text.end());
}

// A minimal but structurally real COMPINSTANCETREE: one assembly swFile
// (id=1, the document itself — always the first <swFile> entry) owning one
// assembly-level <swModel> (swFileRef="1") with three <swReference>
// children:
//   - "part-1": a normal, fully-specified component with a non-identity
//     swTransform.
//   - "part-2": an abbreviated (suppressed) component, missing swSuppressed
//     and swTransform entirely.
//   - "part-3": fully-specified but with a malformed swTransform (wrong
//     token count), which must also fall back to the identity default.
const char kCompInstanceTree[] = R"(<?xml version="1.0" encoding="UTF-8"?>
<swSolidWorks>
  <swHeader>
    <swFile id="1" swDocType="ASSEMBLY" swPath="C:\assembly.SLDASM"/>
    <swFile id="2" swDocType="PART" swPath="C:\part.SLDPRT"/>
  </swHeader>
  <swModelList>
    <swModel id="10" swFileRef="1" swConfigurationId="0" swConfigurationName="Standard">
      <swReference id="20" swName="part-1" swComponentReference=""
          swConfigurationId="0" swConfigurationName="Standard"
          swSuppressed="NO" swHidden="NO" swExcludeFromBOM="NO"
          swModelRef="11"
          swTransform="1 0 0 0 0 1 0 0 0 0 1 0 2 3 4 1" swTransformStamp="1"/>
      <swReference id="21" swName="part-2" swModelRef="11"/>
      <swReference id="22" swName="part-3" swComponentReference=""
          swConfigurationId="0" swConfigurationName="Standard"
          swSuppressed="NO" swHidden="NO" swExcludeFromBOM="NO"
          swModelRef="11"
          swTransform="1 2 3" swTransformStamp="1"/>
    </swModel>
    <swModel id="11" swFileRef="2" swConfigurationId="0" swConfigurationName="Standard"/>
  </swModelList>
</swSolidWorks>
)";

constexpr std::array<double, 16> kIdentityTransform = {
    1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

TEST(XmlReadersTest, ParseComponentsExtractsTransform) {
  const std::vector<Component> components =
      ParseComponents(ToBytes(kCompInstanceTree), /*config_index=*/0);

  ASSERT_EQ(components.size(), 3u);

  const Component& normal = components[0];
  EXPECT_EQ(normal.name, "part-1");
  EXPECT_EQ(normal.path, "C:\\part.SLDPRT");
  const std::array<double, 16> expected = {1, 0, 0, 0, 0, 1, 0, 0,
                                           0, 0, 1, 0, 2, 3, 4, 1};
  EXPECT_EQ(normal.transform, expected);
}

TEST(XmlReadersTest, ParseComponentsDefaultsToIdentityWhenAttributeAbsent) {
  const std::vector<Component> components =
      ParseComponents(ToBytes(kCompInstanceTree), /*config_index=*/0);

  ASSERT_EQ(components.size(), 3u);

  const Component& suppressed = components[1];
  EXPECT_EQ(suppressed.name, "part-2");
  EXPECT_TRUE(suppressed.is_suppressed);
  EXPECT_EQ(suppressed.transform, kIdentityTransform);
}

TEST(XmlReadersTest, ParseComponentsDefaultsToIdentityOnMalformedTransform) {
  const std::vector<Component> components =
      ParseComponents(ToBytes(kCompInstanceTree), /*config_index=*/0);

  ASSERT_EQ(components.size(), 3u);

  const Component& malformed = components[2];
  EXPECT_EQ(malformed.name, "part-3");
  EXPECT_FALSE(malformed.is_suppressed);
  EXPECT_EQ(malformed.transform, kIdentityTransform);
}

TEST(XmlReadersTest, ParseKeywordsAcceptsLocalizedSheetsAndSkipsSheetFormats) {
  const auto sheets = ParseKeywordsXml(ToBytes(R"(<Keywords>
    <Sheet Type="Sheet" Name="English"/>
    <Sheet Type="Blatt" Name="German"/>
    <Sheet Type="List" Name="CzechSlovak"/>
    <Sheet Type="Formát listu" Name="Format"/>
  </Keywords>)"));

  ASSERT_EQ(sheets.size(), 3u);
  EXPECT_EQ(sheets[0].name, "English");
  EXPECT_EQ(sheets[1].name, "German");
  EXPECT_EQ(sheets[2].name, "CzechSlovak");
}

}  // namespace
}  // namespace openswx::internal
