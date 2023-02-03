# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

file(GLOB_RECURSE headers "*.h" "*.hpp")
file(GLOB_RECURSE sources "*.cpp")

dart_build_dartpy_submodule(
  TARGET_NAME dartpy_math
  SOURCES ${sources} ${headers}
  TARGET_LINK_LIBRARIES ${PROJECT_NAME}${DART_MAJOR_VERSION}-math
)
