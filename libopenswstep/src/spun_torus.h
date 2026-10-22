// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_SPUN_TORUS_H_
#define OPENSWSTEP_SRC_SPUN_TORUS_H_

#include "openparasolid/model.h"

namespace openswstep::internal {
struct SpunTorus {
  openparasolid::TorusGeometry geometry;
  bool same_normal;
};
// Recognizes a complete circular generator in a meridian plane of a regular
// ring torus. The normal flag relates SPUN's du x dv to the outward torus
// normal, independently of generator/surface sense metadata. Restricted
// generator domains and nonmeridional circles require separate parameter-domain
// handling.
[[nodiscard]] openparasolid::Result<SpunTorus> RecognizeSpunTorus(
    const openparasolid::SpunSurfaceGeometry& spun,
    const openparasolid::CircleGeometry& circle, double tolerance);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_SPUN_TORUS_H_
