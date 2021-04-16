# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

add_subdirectory(src)
add_subdirectory(tests)

file(GLOB_RECURSE dartc_headers "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
dart_format_add(${dartc_headers})
