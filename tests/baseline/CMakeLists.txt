# Copyright (c) 2011-2025, The DART development contributors
#
# Baseline ODE LCP Solver Library
# This is the original ODE implementation used as ground truth for testing

# Collect source files
file(GLOB baseline_ode_srcs "odelcpsolver/*.cpp")
file(GLOB baseline_ode_hdrs "odelcpsolver/*.h")

# Create a static library for the baseline ODE LCP solver
dart_add_library(dart-baseline-ode ${baseline_ode_srcs} ${baseline_ode_hdrs})

# Set include directories
target_include_directories(dart-baseline-ode
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# This is a baseline library for testing only, not for installation
set_target_properties(dart-baseline-ode PROPERTIES
  EXCLUDE_FROM_ALL FALSE
  FOLDER "Baseline"
)
