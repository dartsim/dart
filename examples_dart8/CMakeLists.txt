# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# DART 8.0 Examples

cmake_minimum_required(VERSION 3.22.1)

# Helper function to add dart8 example
function(dart8_add_example EXAMPLE_NAME)
  add_executable(${EXAMPLE_NAME} ${EXAMPLE_NAME}.cpp)
  target_link_libraries(${EXAMPLE_NAME}
    PRIVATE
      dart8
  )

  # Set example properties
  set_target_properties(${EXAMPLE_NAME} PROPERTIES
    FOLDER "dart8/examples"
  )
endfunction()

# Add examples
dart8_add_example(hello_dart8)
dart8_add_example(utilities_demo)

message(STATUS "dart8: Added 2 examples")
