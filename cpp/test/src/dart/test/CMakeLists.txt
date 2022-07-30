# Copyright (c) 2011-2022, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# TODO(JS): Check dependencies

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}${DART_VERSION_MAJOR}-test)

add_library(${target_name} STATIC ${hdrs} ${srcs})
target_include_directories(${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${DART_SOURCE_DIR}/tests/src>
    $<BUILD_INTERFACE:${DART_BINARY_DIR}/tests/src>
)

# Set link libraries
if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
  target_link_libraries(${target_name} PUBLIC "stdc++fs")
endif()
target_link_libraries(${target_name} PUBLIC gtest gtest_main)

target_compile_features(${target_name} PUBLIC cxx_std_17)

dart_clang_format_add(${hdrs} ${srcs})

add_subdirectory(math)
