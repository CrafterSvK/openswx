// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <fstream>
#include <limits>

#include "openswstep/export.h"
#include "step_writer.h"

namespace openswstep {
openparasolid::Result<StepDocument> ExportPayload(
    const openswx::ParasolidPayload& payload, openparasolid::NodeId body,
    const ExportOptions& options, const openparasolid::SchemaCatalog* catalog) {
  using namespace openparasolid;
  const auto valid = internal::ValidateOptions(options);
  if (!valid.ok()) return Result<StepDocument>::Err(valid.error());
  if (!payload.header_valid || payload.ghost_partition ||
      (payload.kind != openswx::ParasolidPayloadKind::kTransmit &&
       payload.kind != openswx::ParasolidPayloadKind::kPartition))
    return Result<StepDocument>::Err(
        {ErrorCode::kUnsupportedFormat, 0,
         "STEP export requires an explicitly selected ordinary, non-ghost "
         "Parasolid payload"});
  return ExportParasolid(payload.payload.data, body, options, catalog);
}
openparasolid::Result<StepDocument> ExportSolidWorksFile(
    const std::filesystem::path& input, std::size_t payload_index,
    openparasolid::NodeId body, const ExportOptions& options,
    const openparasolid::SchemaCatalog* catalog) {
  using namespace openparasolid;
  const auto valid = internal::ValidateOptions(options);
  if (!valid.ok()) return Result<StepDocument>::Err(valid.error());
  std::error_code error;
  const auto size = std::filesystem::file_size(input, error);
  if (error)
    return Result<StepDocument>::Err(
        {ErrorCode::kIo, 0,
         "Cannot determine SolidWorks input size: " + error.message()});
  if (!options.parsing.max_input_bytes)
    return Result<StepDocument>::Err(
        {ErrorCode::kInvalidOptions, 0, "Input byte budget must be positive"});
  if (size > options.parsing.max_input_bytes)
    return Result<StepDocument>::Err(
        {ErrorCode::kResourceLimit, 0, "SolidWorks input byte limit exceeded"});
  const auto container = openswx::InspectContainer(input);
  if (!container.ok())
    return Result<StepDocument>::Err({ErrorCode::kIo, 0, container.error()});
  const auto extraction = openswx::ExtractParasolidStreams(container.value());
  if (!extraction.issues.empty())
    return Result<StepDocument>::Err(
        {ErrorCode::kInvalidRecord,
         extraction.issues.front().marker_offset.value_or(0),
         "Parasolid extraction: " + extraction.issues.front().message});
  if (payload_index >= extraction.payloads.size())
    return Result<StepDocument>::Err(
        {ErrorCode::kInvalidOptions, 0,
         "Selected Parasolid payload index is out of range"});
  return ExportPayload(extraction.payloads[payload_index], body, options,
                       catalog);
}
openparasolid::Result<uint64_t> WriteStepFile(
    const StepDocument& document, const std::filesystem::path& output) {
  using namespace openparasolid;
  if (document.text.size() >
      static_cast<uint64_t>(std::numeric_limits<std::streamsize>::max()))
    return Result<uint64_t>::Err({ErrorCode::kResourceLimit, 0,
                                  "STEP document exceeds file writer range"});
  std::ofstream file(output, std::ios::binary | std::ios::trunc);
  if (!file)
    return Result<uint64_t>::Err(
        {ErrorCode::kIo, 0, "Cannot open STEP output file"});
  file.write(document.text.data(),
             static_cast<std::streamsize>(document.text.size()));
  file.close();
  if (!file)
    return Result<uint64_t>::Err(
        {ErrorCode::kIo, 0, "Cannot write or close STEP output file"});
  return Result<uint64_t>::Ok(document.text.size());
}
}  // namespace openswstep
