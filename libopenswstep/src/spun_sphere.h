// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#ifndef OPENSWSTEP_SRC_SPUN_SPHERE_H_
#define OPENSWSTEP_SRC_SPUN_SPHERE_H_

#include "openparasolid/model.h"

namespace openswstep::internal {
struct SpunSphere {
  openparasolid::SphereGeometry geometry;
  bool same_normal;
};
// Certifies a circular pole-to-pole meridian centered on the spin axis. The
// whole support is spherical; same_normal relates source du x dv to its outward
// normal. Frame and domain discrepancies must fit the source length tolerance.
[[nodiscard]] openparasolid::Result<SpunSphere> RecognizeSpunSphere(
    const openparasolid::SpunSurfaceGeometry& spun,
    const openparasolid::CircleGeometry& circle, double tolerance);
}  // namespace openswstep::internal
#endif  // OPENSWSTEP_SRC_SPUN_SPHERE_H_
