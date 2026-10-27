// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors

#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "openswx/document.h"

namespace openswx {

/// Drawing coordinates are stored in metres, matching the source XML.
struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Line3D {
  Vec3 start;
  Vec3 end;
  uint64_t source_offset = 0;
  uint8_t source_record_type = 0;
};

/// One cached face mesh recovered from Contents/VBLists. Coordinates remain
/// in the source model unit (metres for the drawing caches observed so far).
/// The triangles are expanded from SolidWorks triangle strips.
struct FaceTessellation {
  uint64_t source_offset = 0;
  /// Exact descriptor-table offset and the nearest preceding model-reference
  /// offset. One face class can contain several view/configuration groups.
  uint64_t source_table_offset = 0;
  uint64_t source_group_offset = 0;
  /// Nearest native uoBodyPropInfo_c group boundary. When no explicit body
  /// boundary precedes this face, this equals source_group_offset.
  uint64_t source_body_group_offset = 0;
  /// Source model stem recovered from the nearest preceding serialized
  /// SLDPRT/SLDASM path. Drawing-view bucket references may append an
  /// internal numeric instance suffix to this value.
  std::string referenced_document;
  /// Native drawing-view owner recovered from the nearby serialized
  /// `component@view` reference when SolidWorks stores one.
  std::string owning_view_name;
  /// Component instance prefix from the same native reference.
  std::string owning_component_name;
  std::vector<Vec3> vertices;
  std::vector<Vec3> normals;
  std::vector<uint32_t> strip_lengths;
  std::vector<std::array<uint32_t, 3>> triangles;
};

/// One cached SolidWorks visual-property definition recovered from
/// Contents/VBLists. SolidWorks COLORREF has already been converted to the
/// canonical 0xRRGGBB representation used by SVG entities.
/// The remaining material fields, including transparency, are not confirmed.
enum class DrawingAppearanceScope {
  kClassDefinition,
  kInlineUnresolved,
};

struct DrawingAppearance {
  std::string name;
  uint32_t rgb = 0xC0C0C0u;
  double opacity = 1.0;
  uint64_t source_offset = 0;
  DrawingAppearanceScope scope = DrawingAppearanceScope::kClassDefinition;
  /// True only after ambient/diffuse/specular material response has been
  /// decoded as well as the base colour.
  bool has_material_response = false;
};

enum class DrawingLightKind {
  kAmbient,
  kDirectional,
};

/// Cached SolidWorks scene-light record. The property order matches the
/// public SolidWorks light API: ambient, diffuse, specular, colour, enabled,
/// fixed, and (for directional lights) X/Y/Z direction.
struct DrawingSceneLight {
  DrawingLightKind kind = DrawingLightKind::kAmbient;
  std::string name;
  double ambient = 0.0;
  double diffuse = 0.0;
  double specular = 0.0;
  uint32_t rgb = 0xFFFFFFu;
  bool enabled = true;
  Vec3 direction;
  uint64_t source_offset = 0;
};

struct LineVisibilitySpan {
  double start_fraction = 0.0;
  double end_fraction = 1.0;
  bool hidden = false;
};

struct Rect2 {
  Vec2 min;
  Vec2 max;
};

enum class LinePattern {
  kSolid,
  kHidden,
  kCenter,
  kChain,
  kChainThick,
  kPhantom,
  kStitch,
};

struct LineStyle {
  uint32_t rgb = 0;
  LinePattern pattern = LinePattern::kSolid;
  int source_line_style = 0;
  int source_line_weight = 0;
};

/// Typed provenance retained by the drawing intermediate representation.
/// Values left at zero (or kNoView) were not recovered from the source.
struct DrawingEntitySource {
  static constexpr uint32_t kNoView = std::numeric_limits<uint32_t>::max();

  uint32_t view_index = kNoView;
  uint64_t record_offset = 0;
  uint64_t topology_component_offset = 0;
  uint8_t record_type = 0;
  uint64_t body_group_offset = 0;
  bool body_group_inferred = false;
};

struct Line2D {
  Vec2 start;
  Vec2 end;
  LineStyle style;
  std::string source_table;
  DrawingEntitySource source;
};

/// A circular display-list arc in sheet coordinates. Positive normal_z means
/// counter-clockwise traversal from start to end in the sheet coordinate system.
struct Arc2D {
  Vec2 start;
  Vec2 end;
  Vec2 center;
  double normal_z = 1.0;
  LineStyle style;
  std::string source_table;
};

struct FilledTriangle2D {
  std::array<Vec2, 3> points;
  uint32_t rgb = 0xD9D9D9u;
  double opacity = 1.0;
  std::string source_table;
};

enum class TextAnchor {
  kStart,
  kMiddle,
  kEnd,
};

struct TextAnnotation {
  std::string text;
  Vec2 origin;
  double rotation_radians = 0.0;
  double height = 0.0;
  double width = 0.0;
  std::string font_family;
  std::string source_table;
  TextAnchor anchor = TextAnchor::kStart;
  // Cached advance of the complete text run. This differs from width, which
  // may describe a containing table cell rather than the rendered glyphs.
  double rendered_width = 0.0;
  /// Binary drawing caches store cap heights; Tables XML uses an em size.
  bool height_is_cap_height = true;
  bool bold = false;
};

/// An RGBA sketch bitmap placed in sheet-space. The serialized placement is
/// the lower-left sheet point; pixel rows still run from the visual top edge.
struct RasterImage2D {
  Vec2 bottom_left;
  double width = 0.0;
  double height = 0.0;
  uint32_t pixel_width = 0;
  uint32_t pixel_height = 0;
  std::vector<uint8_t> rgba;
  std::string source_table;
};

enum class DimensionOrientation {
  kHorizontal,
  kVertical,
};

/// A displayed linear drawing dimension recovered from Definition.
/// measured_value_m is the model-space value; paper_length_m is populated
/// while composing a sheet once the owning drawing-view scale is known.
struct Dimension {
  std::string text;
  Vec2 text_origin;
  double measured_value_m = 0.0;
  double paper_length_m = 0.0;
  DimensionOrientation orientation = DimensionOrientation::kHorizontal;
  uint64_t source_offset = 0;
  std::string source_table;
};

/// A BOM item balloon recovered from the drawing annotation cache. Both
/// points are already in sheet-space metres; the leader anchor references
/// model geometry while text_origin is the displayed item-number centre.
struct BomBalloon {
  std::string text;
  Vec2 text_origin;
  Vec2 leader_anchor;
  uint64_t source_offset = 0;
  std::string source_table;
  /// DisplayLists may already contain the native leader polyline.  In that
  /// case the SVG writer must only add the recovered arrowhead and label.
  bool has_cached_leader = false;
  Vec2 cached_leader_point;
  /// Native displayed text height recovered from the balloon note record.
  /// Zero means that the record layout did not expose a trustworthy metric.
  double text_height_m = 0.0;
};

using DrawingEntity = std::variant<Line2D, Arc2D, FilledTriangle2D,
                                     RasterImage2D, TextAnnotation, Dimension,
                                     BomBalloon>;

enum class DrawingViewKind {
  kStandard,
  kSection,
};

/// One view layer in the drawing IL. Face-group provenance distinguishes a
/// native `component@view` link from the document/bounds fallback.
struct DrawingViewOwnership {
  std::string name;
  std::string referenced_document;
  uint64_t bucket_source_offset = 0;
  uint64_t placement_source_offset = 0;
  /// Face group named by a native `component@view` reference, when present.
  uint64_t native_face_group_offset = 0;
  /// Face group currently selected for rendering.
  uint64_t face_group_offset = 0;
  bool face_group_native = false;
  bool face_group_inferred = false;
  DrawingViewKind kind = DrawingViewKind::kStandard;
  std::string native_face_component;
  bool shaded = false;
  std::vector<uint64_t> face_body_group_offsets;
};

struct DrawingSheetGeometry {
  std::string name;
  std::vector<DrawingEntity> entities;
  double width_m = 0.0;
  double height_m = 0.0;
  /// Document print widths indexed by swLineWeights_e values 0..7.
  std::array<double, 8> line_weights_m{};
  std::vector<DrawingViewOwnership> view_ownership;
};

struct DrawingGeometry {
  std::vector<DrawingSheetGeometry> sheets;
};

/// One cached SolidWorks drawing view recovered from Contents/VBLists.
/// Geometry is projected into the view plane and scaled to paper metres.
struct CachedDrawingView {
  uint64_t source_offset = 0;
  std::string name;
  std::string referenced_document;
  Vec3 direction;
  double scale_denominator = 1.0;
  /// Compact 0x01/0x03 edge records are a suppressed hidden-edge cache. Their
  /// presence is the confirmed signature of a shaded-with-edges drawing view.
  bool has_suppressed_edge_cache = false;
  std::vector<Line3D> source_lines;
  DrawingSheetGeometry geometry;
};

struct DrawingViewPlacement {
  uint64_t source_offset = 0;
  std::string name;
  std::array<double, 9> model_to_view{};
  Vec3 translation;
  double scale = 1.0;
  Rect2 outer_bounds;
  Rect2 clip_bounds;
  /// Serialized sheet-space view reference point. Its exact role is not yet
  /// resolved and it is not necessarily the visual midpoint of outer_bounds.
  Vec2 center;
};

/// Reads the XML stored in the modern swXmlContents/Tables stream.
class DrawingGeometryReader {
 public:
  [[nodiscard]] static Result<DrawingGeometry> ReadTablesXml(
      std::span<const uint8_t> xml);

  /// Reverse-engineered reader for uiViewBucket_c cached edge polylines in the
  /// modern Contents/VBLists stream. Curved edges use the same record as
  /// straight edges with a point count greater than two.
  [[nodiscard]] static Result<std::vector<CachedDrawingView>> ReadViewBuckets(
      std::span<const uint8_t> vb_lists);

  /// Reads uoTempFaceTessData_c descriptor tables from Contents/VBLists.
  /// This is the cached model-face tessellation needed for later hidden-line
  /// classification; it is intentionally not added to sheet output yet.
  [[nodiscard]] static Result<std::vector<FaceTessellation>>
  ReadFaceTessellations(std::span<const uint8_t> vb_lists);

  /// Reads confirmed moVisualProperties_c definitions from Contents/VBLists.
  /// This exposes definitions only; ownership is deliberately left unresolved
  /// when more than one appearance is present.
  [[nodiscard]] static std::vector<DrawingAppearance> ReadAppearances(
      std::span<const uint8_t> vb_lists);

  /// Reads class-level and inline moAmbientLight_c/moDirectionLight_c records
  /// from Contents/VBLists. Values are exposed for diagnostics first.
  [[nodiscard]] static std::vector<DrawingSceneLight> ReadSceneLights(
      std::span<const uint8_t> vb_lists);

  /// Splits an edge into exact visible/hidden parameter intervals by clipping
  /// it against projected tessellation triangles and their depth planes.
  /// View-space positive Z is nearer to the observer, matching the confirmed
  /// SolidWorks drawing-view matrices.
  [[nodiscard]] static std::vector<LineVisibilitySpan>
  SplitLineByTessellation(
      const Line3D& line, std::span<const FaceTessellation> faces,
      const std::array<double, 9>& model_to_view);

  /// Compatibility helper: true when at least half the edge is hidden.
  [[nodiscard]] static bool IsLineHiddenByTessellation(
      const Line3D& line, std::span<const FaceTessellation> faces,
      const std::array<double, 9>& model_to_view);

  /// Clips one projected triangle to a drawing-view rectangle and triangulates
  /// the remaining convex polygon as a fan. Empty output means fully clipped.
  [[nodiscard]] static std::vector<std::array<Vec2, 3>> ClipTriangleToRect(
      const std::array<Vec2, 3>& triangle, const Rect2& bounds);

  /// Reads the confirmed view matrix, scale, placement and clipping rectangle
  /// from Contents/Definition. view_names must come from KeyWords for the
  /// selected drawing sheet.
  [[nodiscard]] static Result<std::vector<DrawingViewPlacement>>
  ReadViewPlacements(std::span<const uint8_t> definition,
                     std::span<const std::string> view_names);

  /// Reads confirmed two-point line commands from Contents/DisplayLists.
  /// The commands already carry sheet-space coordinates and include cached
  /// dimension, extension, leader, arrow and symbol strokes. Ownership is
  /// assigned by the nearest preceding known drawing-view reference.
  [[nodiscard]] static Result<std::vector<Line2D>> ReadDisplayListLines(
      std::span<const uint8_t> display_lists,
      std::span<const std::string> all_view_names,
      std::span<const std::string> selected_view_names,
      double sheet_width_m,
      double sheet_height_m,
      std::span<const uint32_t> selected_object_ids = {},
      std::span<const std::string> all_sheet_names = {},
      std::string_view selected_sheet_name = {});

  /// Reads opcode-128/payload-2 circular arcs. The payload stores sheet-space
  /// start, end and center points followed by the plane normal.
  [[nodiscard]] static Result<std::vector<Arc2D>> ReadDisplayListArcs(
      std::span<const uint8_t> display_lists,
      std::span<const std::string> all_view_names,
      std::span<const std::string> selected_view_names,
      double sheet_width_m,
      double sheet_height_m,
      std::span<const uint32_t> selected_object_ids = {},
      std::span<const std::string> all_sheet_names = {},
      std::string_view selected_sheet_name = {});

  /// Reads confirmed sheet-space text commands from Contents/DisplayLists.
  /// SolidWorks 19000 uses opcode variants ending in decimal 6 with payload
  /// type 18; their common prefix carries origin, rotation and resolved UTF-16
  /// display text. Sheet-end ownership also recovers cached format text.
  [[nodiscard]] static Result<std::vector<TextAnnotation>> ReadDisplayListText(
      std::span<const uint8_t> display_lists,
      std::span<const std::string> all_view_names,
      std::span<const std::string> selected_view_names,
      double sheet_width_m,
      double sheet_height_m,
      std::span<const uint32_t> selected_object_ids = {},
      std::span<const std::string> all_sheet_names = {},
      std::string_view selected_sheet_name = {});

  /// Maps serialized display-object IDs to one Definition sheet range. These
  /// IDs are the authoritative owner for DisplayLists annotation commands;
  /// drawing-view references are only a fallback for records without objID.
  [[nodiscard]] static Result<std::vector<uint32_t>> ReadDisplayObjectIds(
      std::span<const uint8_t> definition,
      std::span<const std::string> sheet_names,
      std::string_view sheet_name);

  /// Reads the confirmed SolidWorks 19000 linear-dimension cache layout.
  /// The result contains displayed text, model-space measurement, orientation
  /// and sheet-space text centre. Extension-line attachment points remain a
  /// discovery item.
  [[nodiscard]] static Result<std::vector<Dimension>> ReadLegacyDimensions(
      std::span<const uint8_t> definition,
      std::span<const std::string> sheet_names,
      std::string_view sheet_name);

  /// Reads the confirmed SolidWorks 19000 moDisplayBalloonNote_c layout.
  /// Sheet ownership follows the serialized sheet boundaries in Definition,
  /// matching the established legacy-dimension reader. Auto balloons owned
  /// by moDisplayMagnetLine_c are rejected until their indirect BOM item text
  /// binding is decoded.
  [[nodiscard]] static Result<std::vector<BomBalloon>> ReadLegacyBomBalloons(
      std::span<const uint8_t> definition,
      std::span<const std::string> sheet_names,
      std::string_view sheet_name);

  /// Reads the confirmed legacy weldment/cut-list table grid stored in
  /// Contents/Definition. The decoder uses the native table anchor, serialized
  /// column widths, row height and displayed item-number cells; cell text is a
  /// separate discovery step.
  [[nodiscard]] static Result<std::vector<Line2D>>
  ReadLegacyWeldmentTableLines(
      std::span<const uint8_t> definition,
      std::span<const std::string> sheet_names,
      std::string_view sheet_name);

  /// Reads visible text cells from the same legacy weldment/cut-list table.
  /// Serialized baselines are matched to the confirmed row-height sequence so
  /// suppressed alternative cut-list rows are not emitted.
  [[nodiscard]] static Result<std::vector<TextAnnotation>>
  ReadLegacyWeldmentTableText(
      std::span<const uint8_t> definition,
      std::span<const std::string> sheet_names,
      std::string_view sheet_name);

  /// Reads the confirmed compact sheet-format sketch layout owned by one
  /// drawing sheet. Point records carry sheet-space coordinates and indexed
  /// segment records reference the zero-based point roster. A physical border
  /// signature separates the format run from other sheet-space sketches.
  [[nodiscard]] static Result<std::vector<Line2D>> ReadSheetFormatLines(
      std::span<const uint8_t> definition,
      std::span<const std::string> sheet_names,
      std::string_view sheet_name,
      double sheet_width_m,
      double sheet_height_m,
      bool suppress_bitmap_helpers = false);

  /// Reads static moDisplayNote_c/moNoteData_c text owned by one drawing
  /// sheet. Property-linked and multiline rich-text notes remain unresolved
  /// and are omitted rather than exposing raw SolidWorks $PRP expressions.
  [[nodiscard]] static Result<std::vector<TextAnnotation>>
  ReadSheetFormatText(std::span<const uint8_t> definition,
                      std::span<const std::string> sheet_names,
                      std::string_view sheet_name);

  /// Reads one raw RGBA Contents/SketchBitmapN stream and its placement from
  /// moSketchBitmapData_c in Definition. Unsupported layouts return no image.
  [[nodiscard]] static Result<std::vector<RasterImage2D>> ReadSketchBitmap(
      std::span<const uint8_t> definition,
      std::span<const uint8_t> sketch_bitmap);

  /// Reads the Thin, Normal and Thick..Thick6 print widths stored in
  /// Contents/Printing. Values are returned in swLineWeights_e order 0..7.
  [[nodiscard]] static Result<std::array<double, 8>> ReadPrintLineWeights(
      std::span<const uint8_t> printing);

  /// Composes one sheet from KeyWords ownership, Definition placements,
  /// VBLists cached edges, optional DisplayLists annotation strokes and
  /// (when supplied) Tables XML entities. An empty sheet_name selects the
  /// first real sheet in KeyWords.
  [[nodiscard]] static Result<DrawingSheetGeometry> ReadCachedSheet(
      std::span<const uint8_t> keywords,
      std::span<const uint8_t> definition,
      std::span<const uint8_t> vb_lists,
      std::span<const uint8_t> tables,
      std::string_view sheet_name,
      std::span<const uint8_t> display_lists = {},
      std::span<const uint8_t> sketch_bitmap = {},
      std::span<const uint8_t> printing = {});
};

}  // namespace openswx
