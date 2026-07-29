# Copyright (c) 2011, The DART development contributors

# The volumetric FEM deformable subsystem is staged internal-only: its sources
# compile into libdart so they can be built and tested in-tree, but no header is
# installed and none is added to the generated dynamics.hpp aggregate. That
# leaves the release branch's public API and ABI surface unchanged while the
# design settles. Install the headers once the subsystem is complete.
#
# This directory is not globbed by dart/dynamics/CMakeLists.txt, so every new
# file must be listed here explicitly or it will silently not build.

set(
  dynamics_fem_headers
  ${CMAKE_CURRENT_SOURCE_DIR}/TetMesh.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DeformableBody.hpp
)

set(
  dynamics_fem_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/TetMesh.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DeformableBody.cpp
)

dart_add_core_headers(${dynamics_fem_headers})
dart_add_core_sources(${dynamics_fem_sources})
