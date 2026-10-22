// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_STEP_WRITER_H_
#define OPENSWSTEP_SRC_STEP_WRITER_H_
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "openswstep/export.h"

namespace openswstep::internal {
using Id = uint64_t;
using openparasolid::ErrorCode;
using openparasolid::ParseError;
using openparasolid::Result;
using openparasolid::Vector3;

[[nodiscard]] Result<bool> ValidateOptions(const ExportOptions& options);
[[nodiscard]] Result<StepDocument> BuildBody(
    const openparasolid::Model& model,
    const openparasolid::BodyGeometryBinding& body,
    const ExportOptions& options);
std::string Reference(Id id);
std::string References(std::span<const Id> ids);
std::string Logical(bool value);
std::string Quote(std::string_view text);

// Retains the first serialization failure. Add returns zero after failure;
// Finish always propagates it and never publishes a partial document.
class StepWriter {
 public:
  explicit StepWriter(const ExportOptions& options)
      : options_(options), uncertainty_(options.tolerance) {}
  void IncludeSourceTolerance(double tolerance);
  Id Add(std::string entity);
  std::string Number(double value);
  Id Point(Vector3 value);
  Id Direction(Vector3 value);
  Id Placement(Vector3 origin, Vector3 normal, Vector3 x_axis);
  [[nodiscard]] Result<StepDocument> Finish(std::string_view representation,
                                            std::span<const Id> items,
                                            ExportStatistics statistics);

 private:
  const ExportOptions& options_;
  std::string entities_;
  uint64_t count_ = 0;
  double uncertainty_;
  std::optional<ParseError> error_;
};
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_STEP_WRITER_H_
