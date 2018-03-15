# Locate yaml-cpp
#
# This module defines
#  YAMLCPP_FOUND - System has yaml-cpp
#  YAMLCPP_INCLUDE_DIRS - The yaml-cpp include directories
#  YAMLCPP_LIBRARIES - The libraries needed to use yaml-cpp
#  YAMLCPP_DEFINITIONS - Compiler switches required for using yaml-cpp
#  YAMLCPP_VERSION - The version of yaml-cpp
#
# By default, the dynamic libraries of yaml-cpp will be found. To find the
# static ones instead, set the YAMLCPP_STATIC_LIBRARY variable to TRUE before
# calling find_package(YamlCpp ...).
#
# If yaml-cpp is not installed in a standard path, you can use the YAMLCPP_DIR
# CMake variable to tell CMake where yaml-cpp is.

# Attempt to find static library first if this is set
if(YAMLCPP_STATIC_LIBRARY)
    set(YAMLCPP_STATIC libyaml-cpp.a)
endif()

# Set up pkg-config to find yaml-cpp.
find_package(PkgConfig)
pkg_check_modules(PC_YAMLCPP QUIET yaml-cpp)
set(YAMLCPP_DEFINITIONS ${PC_YAMLCPP_CFLAGS_OTHER})

# Find the yaml-cpp include directory.
find_path(YAMLCPP_INCLUDE_DIR yaml-cpp/yaml.h
          HINTS ${PC_YAMLCPP_INCLUDEDIR} ${PC_YAMLCPP_INCLUDE_DIRS}
          PATHS ${YAMLCPP_DIR}/include/
          PATH_SUFFIXES include)

# Find the yaml-cpp library.
find_library(YAMLCPP_LIBRARY NAMES ${YAMLCPP_STATIC} yaml-cpp
             HINTS ${PC_YAMLCPP_LIBDIR} ${PC_YAMLCPP_LIBRARY_DIRS}
             PATHS ${YAMLCPP_DIR}/lib/
             PATH_SUFFIXES lib64 lib)

set(YAMLCPP_LIBRARIES ${YAMLCPP_LIBRARY})
set(YAMLCPP_INCLUDE_DIRS ${YAMLCPP_INCLUDE_DIR})

set(YAMLCPP_VERSION ${PC_YAMLCPP_VERSION})

# Handle the QUIETLY and REQUIRED arguments and set YAMLCPP_FOUND to TRUE
# if all listed variables are TRUE.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(YAMLCPP DEFAULT_MSG
	                              YAMLCPP_INCLUDE_DIR YAMLCPP_LIBRARY)
mark_as_advanced(YAMLCPP_INCLUDE_DIR YAMLCPP_LIBRARY)
