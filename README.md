# openswx

A cross-platform toolkit for reading SolidWorks® files (`.SLDPRT`, `.SLDASM`, `.SLDDRW`) without a SolidWorks installation, COM, or any Windows-only dependency.

---

## Overview

The repository contains five components:

```
openswx/
├── libopenparasolid/  C++20 library + binary Parasolid inspector
├── libopenswx/   Pure C++20 library — parses SolidWorks files
├── libopenswstep/ C++20 library — exact STEP export using libopenswx/libopenparasolid
├── libopenbom/   Pure C++20 library — builds hierarchical BOMs (built on libopenswx)
└── asmbox/       HTTP server + CLI tool — built on top of both libraries
```

### libopenparasolid

An independent MIT-licensed C++20 library for extracted binary Parasolid streams.
It reads common/internal headers, schema metadata, fixed-layout record candidates,
stream-local reference candidates and uninterpreted byte ranges. The
`parasolid_inspect` CLI writes JSON. Its `--nodes` mode consumes sequential
records using embedded schema edits and a reviewed administration, attribute, topology
and analytic, spline and procedural geometry subset or an explicit matching
catalog. Candidate
scanning remains an inspection aid;
bounded topology traversal, explicit-plan delta application and exact boundary
binding are available through the library API.
`--geometry` builds an owned reference graph and typed analytic/NURBS objects
for complete partition/transmit streams, with explicit unresolved-reference and
unsupported-geometry diagnostics. See the
[model builder](libopenparasolid/MODEL.md) and
[library documentation](libopenparasolid/README.md) for the API and supported scope.

### libopenswx

A C++20 static library that reads modern SolidWorks files and identifies unsupported legacy containers:

| Format | Versions | Detection |
|--------|----------|-----------|
| Modern chunk format | SW 2015 and later | byte scan for `14 00 06 00 08 00` marker |
| OLE2 Compound Document | SW 2014 and earlier | magic `D0 CF 11 E0 A1 B1 1A E1` |

Extracted data per document:

- **Document type** (part / assembly / drawing)
- **Model version** number
- **Global custom properties** (`docProps/custom.xml`)
- **Document preview** PNG thumbnail
- Per **configuration**: name, custom properties, mass properties (CoG, volume, surface area, mass, inertia tensor), preview PNG, cut-list items (weldments), assembly component references
- Per **drawing sheet**: name, preview PNG, view references

### libopenswstep

An MIT-licensed C++20 static library that writes STEP AP214 files using
`libopenswx` extraction and `libopenparasolid` decoding, topology and geometry
binding. The supported subset covers analytic wire bodies, point-only
acorns, planar polygon sheets with holes and closed planar solids with through
holes. Planar sheets also support exact circle/ellipse disks and circular faces
with round holes. Full cylindrical bands, solid cylinders and tubes retain
analytic surfaces and shared circular boundaries. Complete conical frustums and
conical tubes also retain their analytic surfaces. Source units, payload
selection and body identity are
explicit. Unsupported geometry returns a typed
error; it is never silently omitted or tessellated. See the
[API, supported scope and validation](libopenswstep/README.md).

### libopenbom

A C++20 static library that builds hierarchical, recursive Bills of Materials from SolidWorks files. It resolves Windows component paths to host-filesystem paths, aggregates identical instances, and serialises the result to JSON.

Key types:

| Type | Description |
|------|-------------|
| `BomTransformer` | Entry point — call `Build(path)` to produce a `Bom` |
| `BomTransformerConfig` | Controls configuration selection, filtering, recursion depth, path resolution |
| `PathResolverConfig` | Maps Windows paths to host paths via prefix substitutions and directory search |
| `BomItem` | One row in the BOM tree (name, quantity, properties, children) |
| `Bom` | Root `BomItem` plus accumulated warnings |
| `JsonWriter` | Serialises a `Bom` to JSON string or file |

### asmbox

A Linux HTTP server that lets you upload a ZIP archive of SolidWorks files and browse the extracted metadata, BOM trees, and custom properties in a web UI.  It also ships `swx_dump`, a command-line tool that prints a JSON representation of any single SolidWorks file.

---

## Quick Start with Docker or Podman

If you want to try `asmbox` quickly, the container image is the fastest path.

Build the image locally:

```bash
docker build -t openswx:latest .
```

or:

```bash
podman build -t openswx:latest .
```

Start the web server with persistent data and a bootstrap admin account:

```bash
docker run --rm \
  -p 8087:8087 \
  -v openswx-data:/data \
  -e ASMBOX_ADMIN_USER=admin \
  -e ASMBOX_ADMIN_PASSWORD='change-me-now' \
  openswx:latest
```

With Podman:

```bash
podman run --rm \
  -p 8087:8087 \
  -v openswx-data:/data \
  -e ASMBOX_ADMIN_USER=admin \
  -e ASMBOX_ADMIN_PASSWORD='change-me-now' \
  openswx:latest
```

Open `http://localhost:8087`, sign in as `admin`, and create additional users from the admin area.

Run `swx_dump` directly through `--entrypoint`:

```bash
docker run --rm \
  --entrypoint swx_dump \
  -v "$PWD:/files:ro" \
  openswx:latest \
  /files/example.SLDPRT
```

Inspect or extract every decoded stream in a modern SolidWorks container:

```bash
swx_dump --list-streams /files/example.SLDDRW
swx_dump --dump-streams /tmp/swx-streams /files/example.SLDDRW
swx_dump --dump-unknown /tmp/swx-unknown /files/example.SLDDRW
```

The inspection output includes marker offsets, type IDs, compressed and decoded
sizes, CRC validation, duplicate names, cache cells, tail-directory entries,
unknown marker candidates, and exact unclaimed byte ranges. Container inspection
and drawing SVG export support the modern block format; legacy OLE2 files remain
unsupported by these commands.

The modern parser checks each inline stream's decoded size and CRC before
accepting it. Invalid candidates are skipped and reported by container inspection.

Inspect configuration/partition identities without assuming a current model:

```bash
swx_dump --resolve-saved-geometry "Standard" /files/example.SLDPRT
# Equivalent default; the explicit CLI option takes precedence:
SWX_DUMP_RESOLVE_SAVED_GEOMETRY=Standard swx_dump /files/example.SLDPRT
```

The report separates configuration and partition identity from saved-state and
final-body membership. It records selector evidence, source offsets, cache flags
and per-payload BODY candidates, including sheet bodies. `state_evidence`
compares WORLD/current-PMARK identifiers and validates a bounded structural
history path. Matching identifiers do not by themselves require delta replay and do not
certify final body membership. Automatic delta
reconstruction and final/helper-body classification remain unsupported or
unresolved. Exit zero means the report was produced, not that the model is
current or complete. See the [API and selection contract](libopenswx/SAVED_GEOMETRY.md).

Extract embedded binary Parasolid from a modern part, assembly or drawing:

```bash
swx_dump --extract-parasolid /tmp/part-parasolid /files/example.SLDPRT
swx_dump --extract-parasolid /tmp/assembly-parasolid /files/example.SLDASM
swx_dump --extract-parasolid /tmp/drawing-parasolid /files/example.SLDDRW
```

Use a new or empty output directory. The command searches all active decoded
streams, including configuration partitions, resolved features, drawing
definitions and `SectionBodies`. Historical duplicate chunks are excluded.
It writes separate files under `partitions/`, `deltas/`, `ghost-partitions/`,
`section-bodies/` and `embedded/` as applicable. Binary transmit and partition
payloads use `.x_b`; deltas and payloads with unknown headers use `.bin`.
This extracts existing binary data; it does not generate text `.x_t` files.

`manifest.json` and stdout contain the same JSON report: source stream,
container marker offset, wrapper offset within the decoded stream, compressed
and uncompressed sizes, configuration index/name when available, payload kind,
Parasolid description/schema and decoding issues. Configuration names come from
document metadata; an unavailable name or index is `null`. These legacy fields
are stream-slot lookups, not a cross-family configuration/state certificate;
use `--resolve-saved-geometry` for evidence and independent status fields.

Length-prefixed sections can contain several compressed frames. The exporter
concatenates every frame into one complete payload and rejects the section if
any frame is invalid. `frame_count` and `frames` record each frame's source
header offset and sizes. The payload's compressed/uncompressed sizes are sums
over all frames; `section_offset` identifies the section length field and
`stored_size` includes framing and trailing zero padding. Standalone wrappers
have a `null` section offset. Header parsing accepts variable padding between
the description and the length-prefixed schema token.

Wrapper sizes, zlib checksums and the binary signature are checked. Damaged
blocks are reported without hiding later valid blocks; recoverable payloads
and the manifest are retained, but the command exits with status 1 when issues
occur. No matches produce an empty manifest and status 0. An assembly may
reference external parts without embedding their geometry; this command does
not traverse those references.

The compression marker is also used for non-Parasolid data. Valid unrelated
blocks are skipped and counted as `non_parasolid_block_count`. An unreadable
wrapper is reported even when its content type cannot be determined; a decoding
issue in a third-party stream does not prove that model geometry is damaged.

Extraction does not establish whether a payload is independently readable by
a Parasolid kernel. Delta records require a base; the manifest identifies a
preceding partition in the same stream as a **candidate**, without resolving
dependencies or applying deltas. Ghost partitions remain explicitly separate.
The library exposes the same extraction and diagnostics through
`ExtractParasolidStreams(const ContainerInspection&)` in `openswx/container.h`.

Inspect an extracted binary Parasolid file:

```bash
./build/libopenparasolid/parasolid_inspect --summary /tmp/part-parasolid/partitions/payload-1-offset-4.x_b
./build/libopenparasolid/parasolid_inspect /tmp/part-parasolid/partitions/payload-1-offset-4.x_b > /tmp/inspection.json
./build/libopenparasolid/parasolid_inspect --nodes --summary /tmp/part-parasolid/partitions/payload-1-offset-4.x_b
```

The full report includes record candidates, reference status and uninterpreted
ranges with offsets and byte previews. `--max-records N` bounds the scan. Exit 2
indicates a limited or unsupported candidate scan; exit 0 does not establish
complete topology or a valid saved model. `--nodes` instead reads sequential
schema-defined nodes, reports exact ranges and stops at unknown base types.
Supply `--schema-catalog FILE.json` for additional definitions; without one,
a reviewed 13006 subset is built in. Delta reports include the validated
partition/current-mark pair, bounded reference maps or historical-mark packets,
and supported status-framed node occurrences with their pointer statuses.
Repeated delta indices retain occurrence order. In node mode, `--max-records`,
`--max-fields`, `--max-values` and `--max-delta-entries` set independent resource
bounds; JSON reports the active limits. The built-in catalog includes helical
curve and surface forms. Change application remains unsupported. See the
[inspector documentation](libopenparasolid/README.md).

Export the line and text entities stored in a drawing's XML tables stream:

```bash
swx_dump --export-tables-svg /tmp/tables.svg /files/example.SLDDRW
swx_dump --export-tables-svg /tmp/tables.svg --sheet Sheet1 /files/example.SLDDRW
```

Export the confirmed straight-edge cache of one drawing view from `Contents/VBLists`:

```bash
swx_dump --export-view-svg /tmp/view.svg --view "Drawing View1" /files/example.SLDDRW
```

The reader associates each `uiViewBucket_c` with its stored UTF-16 view reference and projects
confirmed 3D edge endpoint records into the view plane. This diagnostic SVG is auto-cropped.

Compose the first drawing sheet, or select one explicitly, using the view matrices, scales,
placements and clipping rectangles recovered from `Contents/Definition`:

```bash
swx_dump --export-sheet-svg /tmp/sheet.svg /files/example.SLDDRW
swx_dump --export-sheet-svg /tmp/sheet.svg --sheet Sheet1 /files/example.SLDDRW
```

The composed SVG uses the detected physical paper size, view transformations,
scales and clipping, and merges matching entities from `swXmlContents/Tables`.
Supported cached layouts include edge polylines, circular arcs, line patterns
and print widths, text, linear dimensions, BOM balloons, weldment-table lines
and text, sheet-format lines and notes, and sketch bitmaps. Cached face meshes
support shaded views and hidden-line classification. Coverage depends on the
stored record layout; this is not a complete SolidWorks drawing renderer.

Document JSON includes drawing replacement and header reference paths, sheet
preview PNGs when present, and `derived_from_index` for configurations whose
parent is stored in `Contents/CMgrHdr2`. Component placement transforms remain
available in each configuration's component records. Localized drawing sheet
types `Sheet`, `Blatt`, and `List` (English, German, Czech/Slovak) are accepted.

Export SolidWorks' stored full-sheet preview or inspect the binary geometry carriers:

```bash
swx_dump --export-sheet-preview-png /tmp/sheet.png --sheet Sheet1 /files/example.SLDDRW
swx_dump --probe-drawing-geometry /files/example.SLDDRW
```

The geometry probe reports class declarations, sketch-marker families, structurally
recognised coordinate-pair candidates, and their bounds for `Definition`, `DisplayLists`,
and `VBLists`. Candidate coordinates are diagnostics, not rendered entities, until their
view ownership and view-to-sheet transform have been confirmed.

Scan a local CAD archive without uploading files:

```bash
./build/asmbox/swx_scan /path/to/archive
```

or through the container:

```bash
docker run --rm \
  --entrypoint swx_scan \
  -v "$PWD:/files:ro" \
  openswx:latest \
  /files
```

`swx_scan` reads `.SLDPRT`, `.SLDASM`, and `.SLDDRW` files locally, prints only aggregate compatibility statistics, and never uploads CAD content anywhere by itself.

The same container patterns work with `podman`. The image runs as a non-root user, exposes port `8087`, stores persistent state in `/data`, and includes `asmbox`, `swx_dump`, `swx_scan`, and `parasolid_inspect`.

## Help Us Test openswx

Real-world SolidWorks files from different versions, configurations, and modelling workflows are the fastest way to improve parser compatibility.

If you want to help:

- Run `swx_scan` on representative archives and share the anonymized output or JSON report.
- Mention the SolidWorks version, document type, and whether the file parsed successfully.
- Only share real CAD files intentionally; proprietary or confidential models should stay local unless you explicitly want to disclose them.

---

## Prerequisites

| Package | Minimum version | Purpose |
|---------|----------------|---------|
| CMake | 3.20 | Build system |
| GCC or Clang | C++20 capable (GCC 12+) | Compiler |
| zlib (`libz-dev`) | any | Raw-deflate decompression in libopenswx |
| pugixml (`libpugixml-dev`) | 1.11+ | XML parsing in libopenswx |
| nlohmann/json (`nlohmann-json3-dev`) | 3.x | JSON serialization in tools and libopenbom |
| OpenSSL (`libssl-dev`) | 3.x | Password hashing and session token support in asmbox |
| SQLite3 (`libsqlite3-dev`) | 3.x | Result caching and BOM persistence in asmbox |
| libzip (`libzip-dev`) | 1.x | ZIP upload handling in asmbox |

The following are fetched automatically by CMake's `FetchContent` during configuration:

- **Crow** v1.3.3 — HTTP server framework
- **plog** v1.1.10 — Logging
- **googletest** v1.15.2 — Unit tests

On Debian/Ubuntu, install the system packages with:

```bash
sudo apt install cmake build-essential \
    libz-dev libssl-dev libpugixml-dev nlohmann-json3-dev \
    libsqlite3-dev libzip-dev
```

---

## Building

```bash
cd openswx

# Configure (downloads Crow, plog, googletest automatically)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

# Build everything
cmake --build build -j$(nproc)
```

Build outputs include these tools (paths are relative to `build/`):

| Binary | Description |
|--------|-------------|
| `asmbox/asmbox` | HTTP server (default port 8087) |
| `asmbox/swx_dump` | CLI tool — prints JSON to stdout |
| `libopenparasolid/parasolid_inspect` | Inspects extracted binary Parasolid streams |
| `asmbox/swx_scan` | CLI tool — scans directories and writes anonymized aggregate reports |

Run the tests:

```bash
ctest --test-dir build --output-on-failure
```

---

## Tutorial

### 1. Inspect a single file with `swx_dump`

```bash
./build/asmbox/swx_dump /path/to/part.SLDPRT | python3 -m json.tool
```

Example output (truncated):

```json
{
  "doc_type": 1,
  "version": 17000,
  "global_properties": {
    "Desc1": "Winkelstecker",
    "Material": "Messing"
  },
  "configurations": [
    {
      "name": "Standard",
      "properties": { "Configuration": "Standard" },
      "mass_properties": {
        "mass": 0.0251,
        "volume": 2.5e-06,
        "surface_area": 0.00654,
        "center_of_gravity": { "x": -0.003, "y": 0.0, "z": 0.013 }
      }
    }
  ]
}
```

`doc_type` values: `1` = part, `2` = assembly, `3` = drawing.
All mass/geometry values are in SI units (kg, m, m², m³).

---

### 2. Run the asmbox web server

```bash
./build/asmbox/asmbox
```

The server starts at `http://0.0.0.0:8087`.

Open `http://localhost:8087` in a browser to reach the login screen.

`asmbox` is now a multi-user system. Self-service registration is disabled.
The first account must be bootstrapped as an admin, and additional accounts can
then be created, edited, deactivated, or deleted by an admin user.

To bootstrap an admin account on server start, set both
`ASMBOX_ADMIN_USER` and `ASMBOX_ADMIN_PASSWORD`. If the named user already
exists, `asmbox` promotes that user to admin and keeps the existing password.

```bash
ASMBOX_ADMIN_USER=admin \
ASMBOX_ADMIN_PASSWORD='change-me-now' \
  ./build/asmbox/asmbox
```

You can also create or promote an admin explicitly via CLI:

```bash
./build/asmbox/asmbox create-admin admin --password 'change-me-now'
```

If `--password` is omitted, `asmbox` reads the password from
`ASMBOX_ADMIN_PASSWORD` or prompts on stdin.

After signing in, workspaces, profiles, and account data are private to the
signed-in user and remain available across server restarts as long as
`ASMBOX_DATA_DIR` is persistent.

**Upload a ZIP archive** containing one or more SolidWorks files. The server will:
1. Unpack the archive into `ASMBOX_DATA_DIR/users/<user-id>/workspaces/<workspace-id>/files/`
2. Scan for `.SLDPRT`, `.SLDASM`, and `.SLDDRW` files
3. Analyze each file immediately and cache the result in SQLite
4. Return the list of found files as JSON

**Browse the workspace** in the sidebar, click any file to see its parsed document data and properties. After signing out and back in, the workspace list and extracted files are restored from disk.

**View the BOM** via the "Bill of Materials" tab for assembly and drawing files.  The BOM is built recursively by resolving component paths within the workspace directory. If the BOM cannot be derived from the model or input data, the API returns HTTP `422` instead of a generic internal server error.

**Configure properties** per workspace: mark which properties are visible, and assign semantic roles (`part_number`, `description`, `material`, `revision`).  The property designated as `part_number` is used when persisting BOMs.

**Profiles** let you define reusable mapping and transformation rules that are independent of any workspace (they survive workspace deletion and server restarts):
- **Mappings** link SolidWorks custom property names to structured fields in the output (Part, BomItem, or Bom Thrift entities).
- **Rules** transform the BOM tree on the fly:
  - `kaufgruppe` — keep the matched item but remove its children.
  - `non_bom` — exclude the matched item entirely.
  - `phantom` — exclude the matched item and promote its children one level up.

A profile is applied to a BOM request by appending `?profile=<id>` to the URL.

**Resizable panes** — drag the dividers in the Details view, Bill of Materials view, and sidebar to resize panes.  Widths are persisted in `localStorage`.

---

### 3. Scan a directory tree with `swx_scan`

```bash
./build/asmbox/swx_scan /path/to/archive
```

This recursively scans `.SLDPRT`, `.SLDASM`, and `.SLDDRW` files and prints
only aggregate compatibility statistics.  It does not emit filenames, paths,
 custom properties, configuration names, or other CAD metadata.

To write an anonymized JSON report:

```bash
./build/asmbox/swx_scan --report report.json /path/to/archive
```

The JSON report contains only aggregate counters such as total files,
successful parses, failed parses, document types, detected versions, and
stable error categories.

---

### 4. Configuration via environment variables

All settings use the `ASMBOX_` prefix and have sensible defaults:

| Variable | Default | Description |
|----------|---------|-------------|
| `ASMBOX_BIND_ADDR` | `0.0.0.0` | IP address to bind to |
| `ASMBOX_PORT` | `8087` | TCP port |
| `ASMBOX_DATA_DIR` | `$TMPDIR/sw_portal` | Root directory for workspaces and the SQLite database |
| `ASMBOX_TEMPLATE_DIR` | `<binary-dir>/templates` | Directory containing the Crow HTML templates |
| `ASMBOX_ADMIN_USER` | unset | Optional bootstrap admin username |
| `ASMBOX_ADMIN_PASSWORD` | unset | Optional bootstrap admin password |

The main SQLite database (`main.sqlite`) is stored in `ASMBOX_DATA_DIR` and is
persistent across server restarts. Workspace-specific caches are stored in each
workspace directory as `workspace.sqlite`, typically below
`ASMBOX_DATA_DIR/users/<user-id>/workspaces/<workspace-id>/`. The schema is
created with `CREATE TABLE IF NOT EXISTS`, so restarting the server never
erases users, profiles, workspace metadata, or cached analysis data.

Example — run on a non-default port, binding only to localhost, with a persistent data directory:

```bash
ASMBOX_PORT=9090 \
ASMBOX_BIND_ADDR=127.0.0.1 \
ASMBOX_DATA_DIR=/var/lib/asmbox \
ASMBOX_TEMPLATE_DIR=/usr/share/asmbox/templates \
  ./build/asmbox/asmbox
```

---

### 5. Using libopenswx in your own project

Add libopenswx as a subdirectory and link against it:

```cmake
add_subdirectory(path/to/openswx/libopenswx)
target_link_libraries(my_target PRIVATE openswx)
```

Minimal usage:

```cpp
#include "openswx/document.h"

auto result = openswx::SwxDocument::Open("part.SLDPRT");
if (!result.ok()) {
    std::cerr << result.error() << "\n";
    return 1;
}

const openswx::Document& doc = result.value().doc();

for (const auto& cfg : doc.configurations) {
    std::cout << "Config: " << cfg.name << "\n";
    if (cfg.mass_properties) {
        std::cout << "  Mass: " << cfg.mass_properties->mass << " kg\n";
    }
    for (const auto& [key, val] : cfg.properties) {
        std::cout << "  " << key << " = " << val << "\n";
    }
}
```

---

### 6. Using libopenbom in your own project

Add both libraries and link:

```cmake
add_subdirectory(path/to/openswx/libopenswx)
add_subdirectory(path/to/openswx/libopenbom)
target_link_libraries(my_target PRIVATE openbom openswx)
```

Minimal usage:

```cpp
#include "openbom/transformer.h"
#include "openbom/json_writer.h"

openbom::BomTransformerConfig cfg;

// Map Windows component paths to the host filesystem.
cfg.path_resolver.substitutions.push_back(
    {"C:\\Engineering\\Parts\\", "/mnt/nas/parts/"});

// Or simply search a local directory by filename.
cfg.path_resolver.search_dirs.push_back("/data/parts");
cfg.path_resolver.recursive_search = true;
cfg.path_resolver.case_insensitive = true;

openbom::BomTransformer transformer(cfg);
auto result = transformer.Build("/data/top_asm.SLDASM");
if (!result.ok()) {
    std::cerr << result.error() << "\n";
    return 1;
}

const openbom::Bom& bom = result.value();
for (const auto& w : bom.warnings)
    std::cerr << "Warning: " << w << "\n";

openbom::JsonWriter writer;
writer.WriteToFile(bom, "bom.json");
```

`BomTransformerConfig` options:

| Field | Default | Description |
|-------|---------|-------------|
| `configuration_name` | `""` | SolidWorks configuration to activate; empty = first |
| `include_suppressed` | `false` | Include suppressed assembly components |
| `include_hidden` | `false` | Include hidden assembly components |
| `include_exclude_from_bom` | `false` | Include components flagged "exclude from BOM" |
| `expand_cut_lists` | `true` | Add weldment cut-list items as children |
| `max_depth` | `0` | Max recursion depth (0 = unlimited) |
| `path_resolver` | — | `PathResolverConfig` for Windows-to-host path mapping |
| `item_callback` | `nullptr` | Optional post-processing callback per `BomItem` |

---

## REST API reference

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/` | Web UI (`index.html`) |
| `POST` | `/api/auth/login` | Sign in with `{username, password}` |
| `POST` | `/api/auth/logout` | Sign out and clear the session cookie |
| `GET` | `/api/auth/me` | Return the authenticated user |
| `GET` | `/api/account` | Return the current user's profile |
| `PUT` | `/api/account` | Update the current user's username and/or password |
| `GET` | `/api/workspaces` | List the current user's workspaces as JSON |
| `GET` | `/api/workspaces/<id>/files` | Rebuild the file tree for one workspace from disk |
| `DELETE` | `/api/workspaces/<id>` | Delete a workspace and its files |
| `POST` | `/upload` | Upload a `multipart/form-data` request with a `file` field (ZIP) |
| `GET` | `/api/doc/<workspace>/<path_b64>` | Parsed document JSON for one file (cached in SQLite) |
| `GET` | `/api/workspaces/<ws>/props` | Property names + configuration for the workspace |
| `POST` | `/api/workspaces/<ws>/props` | Save property configuration (visibility, roles) |
| `GET` | `/api/profiles` | List all profiles |
| `POST` | `/api/profiles` | Create a profile `{name, description}` |
| `GET` | `/api/profiles/<id>` | Full profile (name, description, mappings, rules) |
| `PUT` | `/api/profiles/<id>` | Save full profile (upsert mappings and rules) |
| `DELETE` | `/api/profiles/<id>` | Delete a profile |
| `GET` | `/api/bom/<workspace>/<path_b64>` | Build BOM; optionally apply profile with `?profile=<id>` |
| `GET` | `/api/admin/users` | List all users; admin only |
| `POST` | `/api/admin/users` | Create a user; admin only |
| `PUT` | `/api/admin/users/<id>` | Update username, admin flag, active flag, or password; admin only |
| `DELETE` | `/api/admin/users/<id>` | Delete a user; admin only |

`<path_b64>` is the relative file path within the workspace, Base64url-encoded.

---

## Architecture notes

```
SolidWorks file
      │
      ▼
libopenswx::ParseFile()
  ├─ IsOle2() → ParseOle2Format()   (OLE2 FAT/directory/sector chain)
  └─ ParseModernFormat()            (marker scan + ROL-decoded names + raw deflate)
      │
      ▼
   StreamMap  (flat map: stream-path → decompressed bytes)
      │
      ├─ ParsePropertyXml()         docProps/custom.xml, Config-N-Properties.xml
      ├─ ParseMassProperties()      docProps/ISolidWorksInformation.xml
      ├─ ParseCutlistXml()          docProps/Config-N-Cutlist-Properties.xml
      ├─ ParseComponents()          swXmlContents/COMPINSTANCETREE
      ├─ ParseSheetNames()          SheetPreviews/SheetNames
      └─ ExtractPng()               PreviewPNG / Config-N-PreviewPNG

libopenbom::BomTransformer::Build()
      │
      ├─ Detect file type (part / assembly / drawing)
      ├─ Select active configuration
      ├─ Recursively expand assembly components
      │    ├─ PathResolver: Windows path → host path
      │    │    ├─ Prefix substitution rules
      │    │    └─ Directory search (shallow + optional recursive)
      │    └─ Aggregate identical instances (quantity > 1)
      └─ Return Bom { root: BomItem, warnings: [...] }

asmbox (server.cc)
      │
      ├─ POST /upload        → unzip, scan, eager-analyze all SWX files
      ├─ GET  /api/doc       → load or analyze + cache in SQLite
      ├─ GET  /api/bom       → BomTransformer::Build() + ApplyBomRules(profile)
      │                        + SaveBom() to SQLite (surrogate-key schema)
      ├─ GET|POST /api/workspaces/<ws>/props  → property config CRUD
      └─ /api/profiles[/<id>]                → profile CRUD (workspace-independent)

SQLite schema (global.sqlite, persistent)
      ├─ workspaces, files, documents, properties
      ├─ boms, bom_items          (surrogate-key BOM persistence)
      ├─ property_configs         (per-workspace property visibility + roles)
      └─ profiles, profile_mappings, profile_rules  (workspace-independent)
```

Both file formats are detected automatically. The library never writes to disk and makes no system calls beyond reading the file.

## Acknowledgements

Many thanks to [CrafterSvK](https://github.com/CrafterSvK) for the contributions
from [their openswx fork](https://github.com/CrafterSvK/openswx): drawing SVG
export and rendering, container inspection and CRC validation, drawing reference
extraction, derived-configuration information, sheet preview serialization, and
Czech/Slovak drawing sheet support.
