# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# TODO(JS): Check dependencies

# Search all header and source files
file(GLOB hdrs "*.h")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}${DART_MAJOR_VERSION}-test)

dart_add_library(${target_name} STATIC ${hdrs} ${srcs})
target_include_directories(${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${DART_SOURCE_DIR}/tests/src>
    $<BUILD_INTERFACE:${DART_BINARY_DIR}/tests/src>
)

# Set link libraries
target_link_libraries(${target_name}
  PUBLIC
    gtest
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-common
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-math
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-optimizer
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
)

target_compile_features(${target_name} PUBLIC cxx_std_17)

dart_format_add(${hdrs} ${srcs})
