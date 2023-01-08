# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

message("[DEBUG] GLUT_LIBRARIES: ${GLUT_LIBRARIES}")

dart_add_component_sub_directory(
  DEPENDENT_PACKAGES_REQUIRED
    GLUT
  TARGET_LINK_LIBRARIES_PUBLIC
    GLUT::GLUT
  TARGET_LINK_LIBRARIES_PRIVATE
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-external-lodepng
)
