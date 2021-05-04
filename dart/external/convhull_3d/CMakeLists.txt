# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Search all header and source files
file(GLOB hdrs "*.h")

# Install
install(
  FILES ${hdrs}
  DESTINATION include/dart/external/convhull_3d
  COMPONENT headers
)
