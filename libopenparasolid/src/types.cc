// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#include "openparasolid/types.h"

#include "openparasolid/result.h"

namespace openparasolid {

const char *ToString(ErrorCode code) noexcept {
  switch (code) {
    case ErrorCode::kIo:
      return "io_error";
    case ErrorCode::kInvalidHeader:
      return "invalid_header";
    case ErrorCode::kUnsupportedFormat:
      return "unsupported_format";
    case ErrorCode::kResourceLimit:
      return "resource_limit";
    case ErrorCode::kInvalidOptions:
      return "invalid_options";
    case ErrorCode::kInvalidSchema:
      return "invalid_schema";
    case ErrorCode::kInvalidRecord:
      return "invalid_record";
    case ErrorCode::kUnsupportedSchema:
      return "unsupported_schema";
  }
  return "unknown";
}
const char *ToString(StreamKind kind) noexcept {
  switch (kind) {
    case StreamKind::kTransmit:
      return "transmit";
    case StreamKind::kPartition:
      return "partition";
    case StreamKind::kDeltas:
      return "deltas";
    case StreamKind::kUnknown:
      return "unknown";
  }
  return "unknown";
}
const char *ToString(RecordKind kind) noexcept {
  switch (kind) {
    case RecordKind::kFaceUse:
      return "face_use";
    case RecordKind::kLoop:
      return "loop";
    case RecordKind::kEdgeUse:
      return "edge_use";
    case RecordKind::kCoedge:
      return "coedge";
    case RecordKind::kVertexUse:
      return "vertex_use";
    case RecordKind::kPoint:
      return "point";
  }
  return "unknown";
}
const char *ToString(ReferenceStatus status) noexcept {
  switch (status) {
    case ReferenceStatus::kSentinel:
      return "sentinel";
    case ReferenceStatus::kUniqueCandidate:
      return "unique_candidate";
    case ReferenceStatus::kAmbiguous:
      return "ambiguous";
    case ReferenceStatus::kUnresolved:
      return "unresolved";
    case ReferenceStatus::kUninterpretedTarget:
      return "uninterpreted_target";
  }
  return "unknown";
}

}  // namespace openparasolid
