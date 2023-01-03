# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

set(CMAKE_DEBUG_POSTFIX "d")

add_library(wamIk SHARED ikfast71.Transform6D.4_6_9_10_11_12_f8.cpp)
target_link_libraries(wamIk PUBLIC dart7-common)
target_compile_definitions(wamIk PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
target_compile_options(wamIk PRIVATE -w)
set_target_properties(wamIk
  PROPERTIES
   LIBRARY_OUTPUT_DIRECTORY "${DART_BINARY_DIR}"
   ARCHIVE_OUTPUT_DIRECTORY "${DART_BINARY_DIR}"
)
