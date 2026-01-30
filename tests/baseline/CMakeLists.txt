# Copyright (c) 2011, The DART development contributors
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
dart_add_library(dart-baseline-ode ${baseline_ode_srcs} ${baseline_ode_hdrs})
set(BUILD_SHARED_LIBS ${_dart_prev_build_shared_libs})
unset(_dart_prev_build_shared_libs)
# Prevent the INTERNAL DART_BUILD_SHARED flag from leaking into consumers.
# Tests link this baseline library alongside the shared DART targets, so we
# strip the interface definition to avoid conflicting values like
# -DDART_BUILD_SHARED=0 and =1 on the same compile command.
set_property(TARGET dart-baseline-ode PROPERTY INTERFACE_COMPILE_DEFINITIONS "")

# Set include directories - use PROJECT_SOURCE_DIR so consumers can include as "tests/baseline/odelcpsolver/..."
target_include_directories(dart-baseline-ode PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}> $<INSTALL_INTERFACE:include>)

# This is a baseline library for testing only, not for installation
set_target_properties(dart-baseline-ode PROPERTIES EXCLUDE_FROM_ALL FALSE FOLDER "Baseline")
