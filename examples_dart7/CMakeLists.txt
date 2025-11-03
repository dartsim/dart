# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# DART 7.0 Examples

cmake_minimum_required(VERSION 3.22.1)

# Helper function to add dart7 example
function(dart7_add_example EXAMPLE_NAME)
  add_executable(${EXAMPLE_NAME} ${EXAMPLE_NAME}.cpp)
  target_link_libraries(${EXAMPLE_NAME}
    PRIVATE
      dart7
  )

  # Set example properties
  set_target_properties(${EXAMPLE_NAME} PROPERTIES
    FOLDER "dart7/examples"
  )
endfunction()

# Add examples
dart7_add_example(hello_dart7)
dart7_add_example(utilities_demo)

message(STATUS "dart7: Added 2 examples")
