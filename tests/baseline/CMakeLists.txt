# Copyright (c) 2011-2025, The DART development contributors
#
# Baseline ODE LCP Solver Library
# This is the original ODE implementation used as ground truth for testing

# Collect source files
file(GLOB baseline_ode_srcs "odelcpsolver/*.cpp")
file(GLOB baseline_ode_hdrs "odelcpsolver/*.h")

# Build the baseline solver as a static library regardless of the global
# BUILD_SHARED_LIBS setting so that the Windows tests can link against the
# resulting .lib directly.
set(_dart_prev_build_shared_libs ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
dart_add_library(
  NAME dart-baseline-ode
  HEADERS ${baseline_ode_hdrs}
  SOURCES ${baseline_ode_srcs}
  PUBLIC_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}
  NO_EXPORT_MACRO
  NO_INSTALL
)
set(BUILD_SHARED_LIBS ${_dart_prev_build_shared_libs})
unset(_dart_prev_build_shared_libs)

# This is a baseline library for testing only, not for installation
set_target_properties(dart-baseline-ode PROPERTIES
  EXCLUDE_FROM_ALL FALSE
  FOLDER "Baseline"
)
