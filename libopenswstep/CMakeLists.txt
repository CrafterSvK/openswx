# SPDX-License-Identifier: MIT
# Copyright (c) 2026 openswx contributors
cmake_minimum_required(VERSION 3.20)
project(openswstep VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  include(CTest)
endif()
if(NOT TARGET openswx)
  add_subdirectory(../libopenswx ${CMAKE_CURRENT_BINARY_DIR}/openswx-build)
endif()
if(NOT TARGET openparasolid)
  add_subdirectory(../libopenparasolid ${CMAKE_CURRENT_BINARY_DIR}/openparasolid-build)
endif()

set(OPENSWSTEP_GENERATED_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated_include)
configure_file(include/openswstep/version.h.in
  ${OPENSWSTEP_GENERATED_INCLUDE_DIR}/openswstep/version.h @ONLY)
add_library(openswstep STATIC src/export.cc src/step_writer.cc src/planar_boundary.cc src/convex_boundary.cc src/conic_boundary.cc src/torus_boundary.cc src/sphere_boundary.cc src/body_writer.cc src/source.cc)
target_compile_features(openswstep PUBLIC cxx_std_20)
target_include_directories(openswstep PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${OPENSWSTEP_GENERATED_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(openswstep PUBLIC openswx openparasolid)
target_compile_options(openswstep PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wconversion -Wshadow -fno-exceptions>
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive- /utf-8>)

if(BUILD_TESTING)
  find_package(ZLIB REQUIRED)
  if(NOT TARGET GTest::gtest_main)
    find_package(GTest REQUIRED)
  endif()
  add_executable(libopenswstep_tests tests/export_test.cc)
  target_link_libraries(libopenswstep_tests PRIVATE openswstep GTest::gtest_main ZLIB::ZLIB)
  include(GoogleTest)
  gtest_discover_tests(libopenswstep_tests)
  set(OPENSWSTEP_OCCT_PYTHON "" CACHE FILEPATH "Optional Python interpreter with OCP for independent STEP round-trip tests")
  if(OPENSWSTEP_OCCT_PYTHON)
    add_executable(openswstep_roundtrip_fixtures tests/roundtrip_fixtures.cc)
    target_link_libraries(openswstep_roundtrip_fixtures PRIVATE openswstep)
    add_test(NAME openswstep_occt_roundtrip
      COMMAND ${OPENSWSTEP_OCCT_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/tests/occt_roundtrip.py
              $<TARGET_FILE:openswstep_roundtrip_fixtures>)
  endif()
endif()
install(TARGETS openswstep EXPORT openswstepTargets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include PATTERN "*.in" EXCLUDE)
install(FILES ${OPENSWSTEP_GENERATED_INCLUDE_DIR}/openswstep/version.h DESTINATION include/openswstep)
install(FILES LICENSE DESTINATION share/openswstep)
