# SPDX-License-Identifier: MIT
# Copyright (c) 2026 openswx contributors

cmake_minimum_required(VERSION 3.20)
project(openparasolid VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OPENPARASOLID_TOOLS_DEFAULT OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(OPENPARASOLID_TOOLS_DEFAULT ON)
  include(CTest)
endif()
option(OPENPARASOLID_BUILD_TOOLS "Build the Parasolid inspector CLI"
       ${OPENPARASOLID_TOOLS_DEFAULT})

set(OPENPARASOLID_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_include")
file(MAKE_DIRECTORY "${OPENPARASOLID_GENERATED_INCLUDE_DIR}/openparasolid")
configure_file(include/openparasolid/version.h.in
  "${OPENPARASOLID_GENERATED_INCLUDE_DIR}/openparasolid/version.h" @ONLY)

add_library(openparasolid STATIC src/header.cc src/inspector.cc src/types.cc src/schema.cc src/delta_packets.cc src/delta_apply.cc src/base_catalog.cc src/geometry_catalog.cc src/spline_catalog.cc src/procedural_catalog.cc src/model.cc src/topology.cc src/brep.cc src/body_binding.cc src/offset_evaluate.cc src/evaluate.cc src/model_evaluate.cc src/nurbs_evaluate.cc src/surface_evaluate.cc src/spun_evaluate.cc src/spun_invert.cc src/swept_evaluate.cc src/normal_evaluate.cc src/chart_parameterize.cc src/surface_invert.cc src/nurbs_invert.cc src/intersection_chart.cc src/intersection_evaluate.cc src/intersection_model.cc src/analytic_model.cc src/spline_model.cc)
target_include_directories(openparasolid
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${OPENPARASOLID_GENERATED_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>)
target_compile_options(openparasolid PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wconversion -Wshadow>
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive- /utf-8>)

if(OPENPARASOLID_BUILD_TOOLS)
  find_package(nlohmann_json CONFIG REQUIRED)
  add_executable(parasolid_inspect tools/parasolid_inspect.cc)
  target_link_libraries(parasolid_inspect PRIVATE openparasolid nlohmann_json::nlohmann_json)
  target_compile_options(parasolid_inspect PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wconversion -Wshadow>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive- /utf-8>)
  install(TARGETS parasolid_inspect RUNTIME DESTINATION bin)
endif()

if(BUILD_TESTING)
  if(NOT TARGET GTest::gtest_main)
    find_package(GTest REQUIRED)
  endif()
  add_executable(libopenparasolid_tests tests/evaluate_test.cc tests/model_evaluate_test.cc tests/trimmed_curve_test.cc tests/spun_surface_test.cc tests/swept_surface_test.cc tests/normal_evaluate_test.cc tests/chart_parameterize_test.cc tests/surface_invert_test.cc tests/nurbs_invert_test.cc tests/intersection_chart_test.cc tests/intersection_evaluate_test.cc tests/intersection_model_test.cc tests/nurbs_evaluate_test.cc tests/surface_evaluate_test.cc tests/nurbs_surface_evaluate_test.cc tests/model_test.cc tests/topology_test.cc tests/edge_topology_test.cc tests/vertex_topology_test.cc tests/brep_reconstruction_test.cc tests/shell_topology_test.cc tests/region_topology_test.cc tests/body_topology_test.cc tests/model_spline_test.cc tests/header_test.cc tests/inspector_test.cc tests/schema_test.cc tests/delta_test.cc tests/delta_packets_test.cc tests/delta_boundary_test.cc tests/delta_status_test.cc tests/embedded_intersection_test.cc tests/attribute_definition_test.cc tests/geometry_test.cc tests/spline_test.cc tests/procedural_test.cc)
  target_link_libraries(libopenparasolid_tests PRIVATE openparasolid GTest::gtest_main)
  if(OPENPARASOLID_BUILD_TOOLS AND UNIX)
    target_sources(libopenparasolid_tests PRIVATE tests/cli_test.cc tests/inspect_options_test.cc)
    target_link_libraries(libopenparasolid_tests PRIVATE nlohmann_json::nlohmann_json)
    target_compile_definitions(libopenparasolid_tests PRIVATE
      PARASOLID_INSPECT_EXECUTABLE="$<TARGET_FILE:parasolid_inspect>")
    add_dependencies(libopenparasolid_tests parasolid_inspect)
  endif()
  include(GoogleTest)
  gtest_discover_tests(libopenparasolid_tests)
endif()

install(TARGETS openparasolid EXPORT openparasolidTargets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include PATTERN "*.in" EXCLUDE)
install(FILES "${OPENPARASOLID_GENERATED_INCLUDE_DIR}/openparasolid/version.h"
  DESTINATION include/openparasolid)
install(FILES LICENSE DESTINATION share/openparasolid)
