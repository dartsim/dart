# Copyright (c) 2011-2022, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# TODO(JS): Remove this
set(DART_BUILD_COMP_external-imgui ON)

# Set basic component settings
set(project_name ${PROJECT_NAME})
set(component_name external-imgui)

# Set target link libraries
set(target_link_libraries_public OpenGL::GL)
if(APPLE)
  set(target_link_libraries_public_skip_checking "-framework Cocoa")
endif()

# Set target compile options
if(CMAKE_COMPILER_IS_GNUCXX)
  set(target_compile_options_private -w)
endif()

# Add component
dart_add_component(
  PROJECT_NAME
    ${PROJECT_NAME}
  PROJECT_VERSION_MAJOR
    ${DART_MAJOR_VERSION}
  PROJECT_SOURCE_DIR
    ${DART_SOURCE_DIR}
  PROJECT_BINARY_DIR
    ${DART_BINARY_DIR}
  COMPONENT_NAME
    ${component_name}
  COMPONENT_DEPENDENCY_PACKAGES
    OpenGL
  TARGET_LINK_LIBRARIES_PUBLIC
    ${target_link_libraries_public}
  TARGET_LINK_LIBRARIES_PUBLIC_SKIP_CHECKING
    ${target_link_libraries_public_skip_checking}
  TARGET_COMPILE_FEATURES_PUBLIC
    cxx_std_17
  TARGET_COMPILE_OPTIONS_PRIVATE
    ${target_compile_options_private}
  TARGET_COMPILE_DEFINITIONS_PUBLIC
    IMGUI_DISABLE_OBSOLETE_FUNCTIONS
)
