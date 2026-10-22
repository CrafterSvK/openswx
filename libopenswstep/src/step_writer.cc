// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "step_writer.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <limits>

#include "openswstep/version.h"

namespace openswstep::internal {
std::string Reference(Id id) { return "#" + std::to_string(id); }
std::string References(std::span<const Id> ids) {
  std::string text = "(";
  bool first = true;
  for (const auto id : ids) {
    if (!first) text += ',';
    first = false;
    text += Reference(id);
  }
  return text + ')';
}
std::string Logical(bool value) { return value ? ".T." : ".F."; }
std::string Quote(std::string_view text) {
  std::string result = "'";
  for (char ch : text) {
    result += ch;
    if (ch == '\'' || ch == '\\') result += ch;
  }
  return result + '\'';
}
Id StepWriter::Add(std::string entity) {
  if (error_) return 0;
  if (count_ >= options_.max_entities) {
    error_ =
        ParseError{ErrorCode::kResourceLimit, 0, "STEP entity limit exceeded"};
    return 0;
  }
  const auto record = Reference(count_ + 1) + "=" + entity + ";\n";
  if (record.size() > options_.max_output_bytes - entities_.size()) {
    error_ = ParseError{ErrorCode::kResourceLimit, 0,
                        "STEP output byte limit exceeded"};
    return 0;
  }
  entities_ += record;
  return ++count_;
}
std::string StepWriter::Number(double value) {
  if (!std::isfinite(value)) {
    if (!error_)
      error_ = ParseError{ErrorCode::kInvalidRecord, 0, "Nonfinite STEP real"};
    return "0.";
  }
  if (value == 0) value = 0;  // Canonicalize negative zero.
  std::array<char, 64> buffer{};
  const auto converted = std::to_chars(
      buffer.data(), buffer.data() + buffer.size(), value,
      std::chars_format::general, std::numeric_limits<double>::max_digits10);
  if (converted.ec != std::errc{}) {
    if (!error_)
      error_ =
          ParseError{ErrorCode::kInvalidRecord, 0, "Cannot format STEP real"};
    return "0.";
  }
  std::string result(buffer.data(), converted.ptr);
  std::replace(result.begin(), result.end(), 'e', 'E');
  if (result.find('.') == std::string::npos) {
    const auto exponent = result.find('E');
    result.insert(exponent == std::string::npos ? result.size() : exponent,
                  ".");
  }
  return result;
}
Id StepWriter::Point(Vector3 value) {
  return Add("CARTESIAN_POINT('',(" + Number(value.x) + ',' + Number(value.y) +
             ',' + Number(value.z) + "))");
}
Id StepWriter::Direction(Vector3 value) {
  return Add("DIRECTION('',(" + Number(value.x) + ',' + Number(value.y) + ',' +
             Number(value.z) + "))");
}
Id StepWriter::Placement(Vector3 origin, Vector3 normal, Vector3 x_axis) {
  const auto point = Point(origin);
  const auto axis = Direction(normal);
  const auto x = Direction(x_axis);
  return Add("AXIS2_PLACEMENT_3D(''," + Reference(point) + ',' +
             Reference(axis) + ',' + Reference(x) + ')');
}
Result<StepDocument> StepWriter::Finish(std::string_view representation,
                                        std::span<const Id> items,
                                        ExportStatistics statistics) {
  const auto application = Add("APPLICATION_CONTEXT('automotive design')");
  Add("APPLICATION_PROTOCOL_DEFINITION('international "
      "standard','automotive_design',2000," +
      Reference(application) + ')');
  const auto product_context =
      Add("PRODUCT_CONTEXT(''," + Reference(application) + ",'mechanical')");
  const auto product = Add("PRODUCT('body'," + Quote(options_.product_name) +
                           ",'',(" + Reference(product_context) + "))");
  Add("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(" + Reference(product) + "))");
  const auto formation =
      Add("PRODUCT_DEFINITION_FORMATION('',''," + Reference(product) + ')');
  const auto definition_context =
      Add("PRODUCT_DEFINITION_CONTEXT('part definition'," +
          Reference(application) + ",'design')");
  const auto definition =
      Add("PRODUCT_DEFINITION('design',''," + Reference(formation) + ',' +
          Reference(definition_context) + ')');
  const auto shape =
      Add("PRODUCT_DEFINITION_SHAPE('',''," + Reference(definition) + ')');
  const auto prefix =
      options_.length_unit == LengthUnit::kMillimetre ? ".MILLI." : "$";
  const auto length = Add(std::string("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(") +
                          prefix + ",.METRE.))");
  const auto angle =
      Add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
  const auto solid_angle =
      Add("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())");
  const auto uncertainty =
      Add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(" +
          Number(options_.tolerance) + ")," + Reference(length) +
          ",'distance_accuracy_value','')");
  const auto context =
      Add("(GEOMETRIC_REPRESENTATION_CONTEXT(3) "
          "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((" +
          Reference(uncertainty) + ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((" +
          Reference(length) + ',' + Reference(angle) + ',' +
          Reference(solid_angle) + ")) REPRESENTATION_CONTEXT('','3D'))");
  const auto representation_id =
      Add(std::string(representation) + '(' + Quote(options_.product_name) +
          ',' + References(items) + ',' + Reference(context) + ')');
  Add("SHAPE_DEFINITION_REPRESENTATION(" + Reference(shape) + ',' +
      Reference(representation_id) + ')');
  if (error_) return Result<StepDocument>::Err(*error_);
  const std::string header =
      "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('Exact source "
      "B-rep'),'2;1');\n"
      "FILE_NAME('model.step','1970-01-01T00:00:00',(''),(''),"
      "'libopenswstep " OPENSWSTEP_VERSION_STRING
      "','openswx','');\n"
      "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n";
  constexpr std::string_view footer = "ENDSEC;\nEND-ISO-10303-21;\n";
  const auto available = options_.max_output_bytes - entities_.size();
  if (header.size() > available || footer.size() > available - header.size())
    return Result<StepDocument>::Err(
        {ErrorCode::kResourceLimit, 0, "STEP output byte limit exceeded"});
  statistics.entities = count_;
  return Result<StepDocument>::Ok(
      {header + entities_ + std::string(footer), statistics});
}
}  // namespace openswstep::internal
