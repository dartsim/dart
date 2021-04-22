# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

set(DART_DARTPY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/dartpy")

add_subdirectory(dartpy)
add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(tutorials)
