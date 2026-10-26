// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENPARASOLID_SRC_EVALUATE_INTERNAL_H_
#define OPENPARASOLID_SRC_EVALUATE_INTERNAL_H_

#include <array>
#include <optional>

#include "openparasolid/evaluate.h"

namespace openparasolid::internal {
[[nodiscard]] std::optional<ParseError> ValidateIntersectionOptions(
    const IntersectionEvaluationOptions& options);
// Validates axis/controls and reads the actual expanded-knot domain.
[[nodiscard]] Result<std::array<double, 2>> NurbsCurveDomain(
    const NurbsCurveGeometry& curve);
// Certifies a positive projected derivative, using derivative control signs or
// outward-rounded local blossom extraction and bounded Bernstein subdivision
// on every original knot span (up to degree seven). Rational curves certify
// the positive A'W-AW' numerator, with arbitrary valid interior multiplicities.
[[nodiscard]] Result<bool> CertifyMonotoneNurbsProjection(
    const NurbsCurveGeometry& curve, std::array<long double, 3> direction);
// Rational curves require degree <= 7 and a positive Bernstein numerator
// A'W-AW' on every span, within a fixed work budget.
// Recovers one parameter of an open, clamped 3D curve only when
// its endpoint-chord projection has a certified strictly positive derivative.
// Bounded bisection preserves original knot units and verifies 3D membership.
[[nodiscard]] Result<double> InvertMonotoneNurbsCurve(
    const NurbsCurveGeometry& curve, Vector3 point, double tolerance);
// Bounds are u-low, u-high, v-low, v-high; validates before extracting knots.
[[nodiscard]] Result<std::array<double, 4>> NurbsSurfaceDomain(
    const NurbsSurfaceGeometry& surface);
[[nodiscard]] Result<SurfaceParameters> InvertNurbsSurface(
    const NurbsSurfaceGeometry& surface, Vector3 point, double tolerance,
    std::optional<SurfaceParameterSeed> seed);
[[nodiscard]] Result<Vector3> UnitCrossProduct(Vector3 first, Vector3 second,
                                              bool reverse);
[[nodiscard]] Result<Vector3> UnitSurfaceNormal(
    const SurfaceEvaluation& sample, bool reverse);
[[nodiscard]] Result<GeometryPayload> ResolveOffsetSurface(
    const Model& model, NodeId surface,
    std::optional<SurfaceParameterSeed> parameters = std::nullopt);
[[nodiscard]] Result<SurfaceParameters> InvertSweptSurface(
    const Model& model, NodeId surface, Vector3 point, double tolerance,
    std::optional<SurfaceParameterSeed> seed = std::nullopt);
[[nodiscard]] Result<SurfaceParameters> InvertSpunSurface(
    const Model& model, NodeId surface, Vector3 point, double tolerance,
    std::optional<SurfaceParameterSeed> seed = std::nullopt);
[[nodiscard]] Result<SurfaceParameters> InvertOffsetSurface(
    const Model& model, NodeId surface, Vector3 point, double tolerance,
    std::optional<SurfaceParameterSeed> seed = std::nullopt);
[[nodiscard]] Result<SurfaceEvaluation> EvaluateOffsetSurface(
    const Model& model, NodeId surface, double u, double v);
// The public dispatchers validate that all parameters are finite.
[[nodiscard]] Result<CurveEvaluation> EvaluateNurbsCurve(
    const NurbsCurveGeometry& curve, double parameter);
[[nodiscard]] Result<SurfaceEvaluation> EvaluateNurbsSurface(
    const NurbsSurfaceGeometry& surface, double u, double v);
[[nodiscard]] Result<SurfaceEvaluation> EvaluateSpunProfile(
    const SpunSurfaceGeometry& surface, const CurveEvaluation& profile,
    double v);
[[nodiscard]] Result<SurfaceEvaluation> EvaluateSweptSection(
    const SweptSurfaceGeometry& surface, const CurveEvaluation& section,
    double v);
}  // namespace openparasolid::internal

#endif  // OPENPARASOLID_SRC_EVALUATE_INTERNAL_H_
