# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

add_library(dartc STATIC ${hdrs} ${srcs})
target_link_libraries(dartc
  PUBLIC
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation
)

set_property(TARGET dartc PROPERTY C_STANDARD 11)

dart_format_add(${hdrs} ${srcs})
