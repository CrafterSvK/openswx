// SPDX-License-Identifier: MIT
// Copyright (c) 2026 openswx contributors
#pragma once
#include "openparasolid/schema.h"
namespace openparasolid::internal {
void AddValueAndTopologyDefinitions(SchemaCatalog& catalog);
void AddGeometryDefinitions(SchemaCatalog& catalog);
void AddSplineAndListDefinitions(SchemaCatalog& catalog);
void AddProceduralDefinitions(SchemaCatalog& catalog);
}  // namespace openparasolid::internal
