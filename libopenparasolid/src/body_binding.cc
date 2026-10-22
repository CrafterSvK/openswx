// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "evaluate_internal.h"
#include "openparasolid/brep.h"

namespace openparasolid {
namespace {
ParseError Error(const char* message, ErrorCode code) {
  return {code, 0, message};
}
// Compare endpoints in natural edge direction, never normalized midpoints:
// different fin curves may use unrelated parameterizations of the same edge.
std::optional<ParseError> CheckSharedEndpoints(
    const Model& model, NodeId body, const std::vector<FaceTrimming>& faces,
    double tolerance) {
  struct Reference {
    const BoundFinGeometry* fin;
    std::optional<std::array<Vector3, 2>> endpoints;
  };
  std::unordered_map<uint32_t, Reference> references;
  auto endpoints =
      [&](const BoundFinGeometry& fin) -> Result<std::array<Vector3, 2>> {
    std::array<Vector3, 2> positions;
    const bool forward = fin.use.sense == TopologySense::kForward;
    for (std::size_t i = 0; i < positions.size(); ++i) {
      const double parameter = (i == 0) == forward ? fin.start : fin.end;
      const auto sample =
          fin.intersection
              ? EvaluateCurve(model, fin.curve, parameter, *fin.intersection)
              : EvaluateCurve(model, fin.curve, parameter);
      if (!sample.ok())
        return Result<std::array<Vector3, 2>>::Err(sample.error());
      positions[i] = sample.value().position;
    }
    return Result<std::array<Vector3, 2>>::Ok(positions);
  };
  for (const auto& face : faces)
    for (const auto& loop : face.loops)
      for (const auto& fin : loop.fins) {
        // Ring seams need not coincide in the absence of recorded vertices.
        if (!fin.use.start_vertex.value || !fin.use.end_vertex.value) continue;
        auto [entry, inserted] = references.try_emplace(
            fin.use.edge.value, Reference{&fin, std::nullopt});
        if (inserted) continue;
        auto& reference = entry->second;
        if (!reference.endpoints) {
          const auto sample = endpoints(*reference.fin);
          if (!sample.ok()) return sample.error();
          reference.endpoints = sample.value();
        }
        const auto sample = endpoints(fin);
        if (!sample.ok()) return sample.error();
        for (std::size_t i = 0; i < sample.value().size(); ++i) {
          const auto& a = (*reference.endpoints)[i];
          const auto& b = sample.value()[i];
          if (std::hypot(a.x - b.x, a.y - b.y, a.z - b.z) > tolerance) {
            auto error = Error("Shared edge fin endpoints exceed tolerance",
                               ErrorCode::kInvalidRecord);
            const auto* node = model.FindNode(fin.use.fin);
            if (node) error.offset = node->source_offset;
            error.message = "Body " + std::to_string(body.value) + ", edge " +
                            std::to_string(fin.use.edge.value) + ", fins " +
                            std::to_string(reference.fin->use.fin.value) + "/" +
                            std::to_string(fin.use.fin.value) + ": " +
                            error.message;
            return error;
          }
        }
      }
  return std::nullopt;
}
Result<BodyFaceBinding> Bind(
    const Model& model, NodeId body,
    std::optional<std::span<const FinParameterInterval>> intervals,
    double tolerance, const BodyFaceBindingOptions& options, bool partial) {
  if (!std::isfinite(tolerance) || tolerance < 0 || !options.max_faces ||
      !options.max_fins || !options.face.max_loops || !options.face.max_fins)
    return Result<BodyFaceBinding>::Err(
        Error("Invalid body face binding options", ErrorCode::kInvalidOptions));
  if (options.intersection) {
    if (auto error =
            internal::ValidateIntersectionOptions(*options.intersection))
      return Result<BodyFaceBinding>::Err(*error);
  }
  std::unordered_map<uint32_t, FinParameterInterval> parameters;
  if (intervals) {
    if (intervals->size() > options.max_fins)
      return Result<BodyFaceBinding>::Err(
          Error("Body interval limit", ErrorCode::kResourceLimit));
    for (const auto& p : *intervals) {
      if (!std::isfinite(p.start) || !std::isfinite(p.end) ||
          p.start == p.end || !parameters.emplace(p.fin.value, p).second)
        return Result<BodyFaceBinding>::Err(
            Error("Duplicate or invalid body fin interval",
                  ErrorCode::kInvalidOptions));
    }
  }
  auto topology = ValidateBodyTopology(model, body, options.topology);
  if (!topology.ok()) return Result<BodyFaceBinding>::Err(topology.error());
  std::unordered_set<uint32_t> seen;
  std::vector<const FaceBoundary*> faces;
  for (const auto& region : topology.value().boundary.regions)
    for (const auto& shell : region.shells)
      for (const auto& face : shell.faces)
        if (seen.insert(face.face.value).second) {
          if (faces.size() == options.max_faces)
            return Result<BodyFaceBinding>::Err(
                Error("Body face binding limit", ErrorCode::kResourceLimit));
          faces.push_back(&face);
        }
  std::sort(faces.begin(), faces.end(),
            [](const auto* a, const auto* b) { return a->face < b->face; });
  if (!parameters.empty()) {
    std::size_t listed = 0;
    for (const auto* face : faces)
      for (const auto& loop : face->loops)
        for (const auto& fin : loop.fins)
          if (parameters.contains(fin.fin.value)) ++listed;
    if (listed != parameters.size())
      return Result<BodyFaceBinding>::Err(
          Error("Intervals reference fins outside selected body faces",
                ErrorCode::kInvalidOptions));
  }
  uint64_t remaining = options.max_fins;
  std::vector<FaceTrimming> bindings;
  for (const auto* face : faces) {
    std::vector<FinParameterInterval> local;
    for (const auto& loop : face->loops) {
      const auto count = loop.fins.size() + (loop.isolated_vertex ? 1u : 0u);
      if (count > remaining)
        return Result<BodyFaceBinding>::Err(Error(
            "Aggregate body binding fin limit", ErrorCode::kResourceLimit));
      remaining -= count;
      if (intervals)
        for (const auto& fin : loop.fins) {
          const auto found = parameters.find(fin.fin.value);
          if (found == parameters.end()) {
            if (partial) continue;
            return Result<BodyFaceBinding>::Err(
                Error("Missing body fin interval", ErrorCode::kInvalidOptions));
          }
          local.push_back(found->second);
          parameters.erase(found);
        }
    }
    auto binding = [&] {
      if (partial)
        return BindFaceTrimmingWithOverrides(model, face->face, local,
                                             tolerance, options.face,
                                             options.intersection);
      if (intervals)
        return BindFaceTrimming(model, face->face, local, tolerance,
                                options.face, options.intersection);
      return BindFaceTrimming(model, face->face, tolerance, options.face,
                              options.intersection);
    }();
    if (!binding.ok()) {
      auto error = binding.error();
      error.message = "Body " + std::to_string(body.value) + ", face " +
                      std::to_string(face->face.value) + ": " + error.message;
      return Result<BodyFaceBinding>::Err(std::move(error));
    }
    bindings.push_back(std::move(binding).value());
  }
  if (!parameters.empty())
    return Result<BodyFaceBinding>::Err(
        Error("Intervals reference fins outside selected body faces",
              ErrorCode::kInvalidOptions));
  if (auto error = CheckSharedEndpoints(model, body, bindings, tolerance))
    return Result<BodyFaceBinding>::Err(std::move(*error));
  return Result<BodyFaceBinding>::Ok(
      {std::move(topology).value(), std::move(bindings)});
}
}  // namespace
Result<BodyFaceBinding> BindBodyFaces(
    const Model& model, NodeId body,
    std::span<const FinParameterInterval> intervals, double tolerance,
    const BodyFaceBindingOptions& options) {
  return Bind(model, body, intervals, tolerance, options, false);
}
Result<BodyFaceBinding> BindBodyFaces(const Model& model, NodeId body,
                                      double tolerance,
                                      const BodyFaceBindingOptions& options) {
  return Bind(model, body, std::nullopt, tolerance, options, false);
}
Result<BodyFaceBinding> BindBodyFacesWithOverrides(
    const Model& model, NodeId body,
    std::span<const FinParameterInterval> overrides, double tolerance,
    const BodyFaceBindingOptions& options) {
  return Bind(model, body, overrides, tolerance, options, true);
}
}  // namespace openparasolid
