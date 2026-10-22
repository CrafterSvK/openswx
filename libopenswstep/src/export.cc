// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include "openswstep/export.h"

#include <algorithm>
#include <cmath>

#include "step_writer.h"

namespace openswstep {
namespace internal {
Result<bool> ValidateOptions(const ExportOptions& options) {
  if ((options.length_unit != LengthUnit::kMetre &&
       options.length_unit != LengthUnit::kMillimetre) ||
      !std::isfinite(options.tolerance) || options.tolerance <= 0 ||
      options.max_entities == 0 || options.max_output_bytes == 0 ||
      options.max_polygon_checks == 0 || options.product_name.size() > 1024 ||
      !std::all_of(options.product_name.begin(), options.product_name.end(),
                   [](unsigned char ch) { return ch >= 32 && ch <= 126; }))
    return Result<bool>::Err({ErrorCode::kInvalidOptions, 0,
                              "STEP export requires explicit length units, "
                              "positive finite tolerance and budgets, and an "
                              "ASCII product name (maximum 1024 bytes)"});
  return Result<bool>::Ok(true);
}
}  // namespace internal
openparasolid::Result<StepDocument> ExportBody(
    const openparasolid::Model& model, openparasolid::NodeId body,
    const ExportOptions& options) {
  using openparasolid::Result;
  const auto valid = internal::ValidateOptions(options);
  if (!valid.ok()) return Result<StepDocument>::Err(valid.error());
  auto bound = openparasolid::BindBodyGeometryWithOverrides(
      model, body, options.fin_intervals, options.wire_intervals,
      options.tolerance, options.binding);
  if (!bound.ok()) return Result<StepDocument>::Err(bound.error());
  return internal::BuildBody(model, bound.value(), options);
}
openparasolid::Result<StepDocument> ExportParasolid(
    std::span<const uint8_t> data, openparasolid::NodeId body,
    const ExportOptions& options, const openparasolid::SchemaCatalog* catalog) {
  using namespace openparasolid;
  const auto valid = internal::ValidateOptions(options);
  if (!valid.ok()) return Result<StepDocument>::Err(valid.error());
  const auto decoded = SchemaParser::Parse(data, catalog, options.parsing);
  if (!decoded.ok()) return Result<StepDocument>::Err(decoded.error());
  if (decoded.value().header.kind != StreamKind::kTransmit &&
      decoded.value().header.kind != StreamKind::kPartition)
    return Result<StepDocument>::Err(
        {ErrorCode::kUnsupportedFormat, 0,
         "STEP export requires an ordinary transmit/partition, without "
         "unapplied history"});
  if (!decoded.value().complete) {
    if (!decoded.value().diagnostics.empty())
      return Result<StepDocument>::Err(
          decoded.value().diagnostics.front().error);
    return Result<StepDocument>::Err({ErrorCode::kInvalidRecord,
                                      decoded.value().decoded_end,
                                      "Incomplete Parasolid stream"});
  }
  auto model = ModelBuilder::Build(decoded.value(), options.model);
  if (!model.ok()) return Result<StepDocument>::Err(model.error());
  return ExportBody(model.value(), body, options);
}
}  // namespace openswstep
