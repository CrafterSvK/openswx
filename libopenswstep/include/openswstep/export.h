// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_EXPORT_H_
#define OPENSWSTEP_EXPORT_H_

/// @file
/// @brief Exact STEP export of selected SolidWorks/Parasolid body geometry.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

#include "openparasolid/brep.h"
#include "openparasolid/schema.h"
#include "openswx/container.h"

/// @brief Bounded, kernel-independent STEP export using openswx and
/// openparasolid. Input models and payloads are borrowed during calls; output
/// documents own all data. Units and stream/body selection are always explicit.
namespace openswstep {
/// @brief Explicit source length unit; coordinates are retained without
/// scaling.
enum class LengthUnit {
  kUnspecified,  ///< Invalid until the caller supplies the source unit.
  kMetre,        ///< Coordinates and tolerances are expressed in metres.
  kMillimetre,   ///< Coordinates and tolerances are expressed in millimetres.
};

/// @brief Caller settings and positive resource budgets for one selected body.
struct ExportOptions {
  /// Required source unit. No unit is guessed from model coordinates.
  LengthUnit length_unit = LengthUnit::kUnspecified;
  /// Positive absolute binding/STEP uncertainty tolerance in source length
  /// units.
  double tolerance = 1e-7;
  /// Product name, printable ASCII, at most 1024 bytes. Quotes are escaped.
  std::string product_name = "SolidWorks body";
  /// Aggregate cap on generated STEP instances, including product metadata.
  uint64_t max_entities = 1000000;
  /// Cap on the complete owned Part 21 output, including its header.
  uint64_t max_output_bytes = 128u * 1024u * 1024u;
  /// Shared boundary-check budget across the selected body: segment,
  /// containment, separation, convex corners, curved hole/edge pairs, cut pairs
  /// and spherical cap selections. Each cylindrical/conical patch or annular
  /// sector consumes five checks.
  uint64_t max_polygon_checks = 1000000;
  /// Limits for sequential decoding of an extracted binary payload.
  openparasolid::SchemaLimits parsing;
  /// Limits for building the source model graph.
  openparasolid::ModelOptions model;
  /// Limits and optional intersection settings for complete body binding.
  openparasolid::BodyGeometryBindingOptions binding;
  /// Optional directed fin intervals; unspecified fins use inference.
  std::vector<openparasolid::FinParameterInterval> fin_intervals;
  /// Optional directed natural-edge wire intervals; other wires use inference.
  std::vector<openparasolid::WireParameterInterval> wire_intervals;
};

/// @brief Export counts for a successfully serialized complete selected body.
struct ExportStatistics {
  uint64_t entities = 0;  ///< Number of generated Part 21 instances.
  uint64_t vertices =
      0;               ///< STEP vertices, including synthetic conic ring seams.
  uint64_t edges = 0;  ///< Distinct exported topological edges.
  uint64_t faces =
      0;  ///< Exported planar, cylindrical, conical or toroidal faces.
  uint64_t shells = 0;  ///< Exported sheet or solid shells.
  uint64_t solids = 0;  ///< Exported manifold solid representations.
};

/// @brief Owned ISO 10303-21/AP214 document; no source model lifetime
/// dependency.
struct StepDocument {
  std::string text;             ///< Complete STEP text, using LF line endings.
  ExportStatistics statistics;  ///< Counts of the serialized representation.
};

/// @brief Binds and exports one body in source coordinates, without
/// tessellation. Supports LINE/CIRCLE/ELLIPSE wire bodies, point-only acorns,
/// and planar polygon faces with LINE edges, one outer loop and disjoint,
/// nonnested polygonal and full-circle hole loops. Source loop order does not
/// determine the outer loop.
/// Planar faces also support one-edge full CIRCLE/ELLIPSE disks and circular
/// faces with disjoint polygonal and/or circular holes. Complete CYLINDER/CONE
/// bands use two coaxial full-circle loops at distinct axial levels and
/// positive cut radii. CYLINDER/CONE also support a single four-edge loop with
/// two coaxial circular arcs and two straight LINE generators, spanning less
/// than one full turn. Generators follow the respective cylinder/cone profile.
/// Exact transverse and axial flux terms are retained for solid patches.
/// SPHERE faces use one full latitude circle for a cap or two full coaxial
/// latitude circles for a band, away from the poles. Winding selects either
/// cap, including caps larger than a hemisphere. Regular ring TORUS bands use
/// two coaxial full-circle cuts; winding selects the meridian interval,
/// including its periodic seam and complementary band. Planar faces also
/// support a single convex LINE/CIRCLE/ELLIPSE contour, including split conics,
/// elliptic arcs and two-edge segments/lenses. Convex LINE/CIRCLE outer
/// contours also support complete circular holes, checked analytically against
/// all supporting tangent half-planes and other holes, with two tolerances of
/// clearance. Each hole/outer-edge and hole/hole pair consumes one check.
/// Planar annular sectors support
/// two concentric circular arcs joined by two radial lines, including major
/// sectors. Other nonconvex planar curved contours, holes in elliptic/nonconvex
/// curved contours, polygonal/multi-edge holes in curved contours, other
/// non-planar trimming contours, spherical pole cuts, apex
/// loops, explicit seam fins, horn/apple/lemon tori and elliptical holes remain
/// unsupported. Sheet orientation is propagated across shared edges. Solid
/// export requires one material region with one closed, connected shell,
/// consistent face-side uses and positive signed volume. Foreign geometry,
/// tolerant fin curves, isolated face loops, compound bodies, general bodies
/// and other curve/surface types return explicit errors. Existing finite
/// membership checks are retained; no global intersection or full CAD-kernel
/// validity certificate is implied.
/// @see Parasolid XT Format Reference (July 2022), 4.3.3-4.3.10 (topology),
/// 5.2.1.1-5.2.1.3 (line/circle/ellipse), 5.2.2.1-5.2.2.3
/// (plane/cylinder/cone), 5.2.2.4 (sphere), 5.2.2.5 (torus), 5.2.8 (senses)
/// and 5.3.4.1 (geometry attachment).
/// @param model Borrowed model with stream-local node identities.
/// @param body Explicit BODY identity in this model.
/// @param options Units, tolerances and bounded work/storage settings.
/// @return Complete owned document, or ParseError with source context. No
/// partial document is returned. Calls on independent inputs may run
/// concurrently.
[[nodiscard]] openparasolid::Result<StepDocument> ExportBody(
    const openparasolid::Model& model, openparasolid::NodeId body,
    const ExportOptions& options);

/// @brief Sequentially decodes a complete ordinary transmit/partition then
/// exports.
/// @param data Borrowed extracted binary stream, starting at its prologue.
/// @param body Explicit stream-local BODY identity.
/// @param options Export, parser and model budgets.
/// @param catalog Optional matching reviewed base schema; null uses the
/// built-in subset.
/// @return Complete document or explicit failure. Unapplied deltas and
/// incomplete reports are rejected, preserving the first sequential diagnostic
/// if present.
[[nodiscard]] openparasolid::Result<StepDocument> ExportParasolid(
    std::span<const uint8_t> data, openparasolid::NodeId body,
    const ExportOptions& options,
    const openparasolid::SchemaCatalog* catalog = nullptr);

/// @brief Exports an explicitly selected libopenswx extraction payload.
/// Ghost/history payloads and unknown kinds are rejected; no history is
/// guessed.
/// @param payload Borrowed extraction result; its bytes are decoded
/// independently.
/// @param body Explicit BODY identity within this payload.
/// @param options Export and source decoding settings.
/// @param catalog Optional reviewed schema catalog for this payload.
/// @return Complete document or explicit failure; no assembly placement is
/// applied.
[[nodiscard]] openparasolid::Result<StepDocument> ExportPayload(
    const openswx::ParasolidPayload& payload, openparasolid::NodeId body,
    const ExportOptions& options,
    const openparasolid::SchemaCatalog* catalog = nullptr);

/// @brief File adapter: inspects a modern SolidWorks container and selects a
/// payload.
/// @param input Source file. Size is bounded by options.parsing.max_input_bytes
/// before invoking libopenswx; its container/extraction checks remain in force.
/// @param payload_index Zero-based index in ExtractParasolidStreams().payloads;
/// no configuration, body, component placement or active payload is guessed.
/// @param body Explicit BODY identity within the selected payload.
/// @param options Export and source decoding settings.
/// @param catalog Optional matching reviewed schema catalog.
/// @return Complete document or explicit failure. Extraction issues are errors.
[[nodiscard]] openparasolid::Result<StepDocument> ExportSolidWorksFile(
    const std::filesystem::path& input, std::size_t payload_index,
    openparasolid::NodeId body, const ExportOptions& options,
    const openparasolid::SchemaCatalog* catalog = nullptr);

/// @brief File adapter writing a complete in-memory document, replacing the
/// target. Serialize first so unsupported inputs never open/truncate the output
/// file. An I/O error may leave a partial file and must be handled by the
/// caller.
/// @param document Previously generated complete STEP document.
/// @param output Destination path; parent directories must already exist.
/// @return Written byte count, or kIo on open/write/close failure.
[[nodiscard]] openparasolid::Result<uint64_t> WriteStepFile(
    const StepDocument& document, const std::filesystem::path& output);
}  // namespace openswstep
#endif  // OPENSWSTEP_EXPORT_H_
