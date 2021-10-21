# Copyright (c) 2011-2021, The DART development contributors

find_package(imgui CONFIG QUIET)

if(imgui_FOUND)
  return()
endif()

find_package(PkgConfig REQUIRED)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_IMGUI imgui QUIET)

# Include directories
# find_path(
#   IMGUI_INCLUDE_DIRS
#   NAMES imgui/imgui.h
#   HINTS ${PC_IMGUI_INCLUDEDIR}
#   PATHS "${CMAKE_INSTALL_PREFIX}/include"
# )
find_path(
  IMGUI_INCLUDE_DIRS
  NAMES imgui.h
  HINTS ${PC_IMGUI_INCLUDEDIR}
  PATHS "${CMAKE_INSTALL_PREFIX}/include"
)

# Libraries
# Give explicit precedence to ${PC_IMGUI_LIBDIR}
find_library(
  IMGUI_LIBRARIES
  NAMES imgui
  HINTS ${PC_IMGUI_LIBDIR}
  NO_DEFAULT_PATH
  NO_CMAKE_PATH
  NO_CMAKE_ENVIRONMENT_PATH
  NO_SYSTEM_ENVIRONMENT_PATH
)

# Version
if(PC_IMGUI_VERSION)
  set(IMGUI_VERSION ${PC_IMGUI_VERSION})
endif()

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  imgui
  FAIL_MESSAGE  DEFAULT_MSG
  REQUIRED_VARS IMGUI_INCLUDE_DIRS IMGUI_LIBRARIES
  VERSION_VAR   IMGUI_VERSION
)

# Define target imgui::imgui
if(imgui_FOUND AND NOT TARGET imgui::imgui)
  add_library(imgui::imgui INTERFACE IMPORTED)
  set_target_properties(imgui::imgui PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${IMGUI_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${IMGUI_LIBRARIES}"
  )
endif()
