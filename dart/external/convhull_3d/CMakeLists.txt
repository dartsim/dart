# Copyright (c) 2011-2022, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Search all header and source files
file(GLOB hdrs "*.h")

# Install
install(
  FILES ${hdrs}
  DESTINATION include/${org_name}/${PROJECT_NAME}${DART_MAJOR_VERSION}/${PROJECT_NAME}/external/convhull_3d
  COMPONENT headers
)