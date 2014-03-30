# Find NLOPT
#
# This sets the following variables:
# NLOPT_FOUND
# NLOPT_INCLUDE_DIRS
# NLOPT_LIBRARIES
# NLOPT_DEFINITIONS

find_package(PkgConfig QUIET)
pkg_check_modules(PC_NLOPT nlopt)
set(NLOPT_DEFINITIONS ${PC_NLOPT_CFLAGS_OTHER})

find_path(NLOPT_INCLUDE_DIR nlopt.h
    HINTS ${PC_NLOPT_INCLUDEDIR} ${PC_NLOPT_INCLUDE_DIRS}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

find_library(NLOPT_LIBRARY NAMES nlopt nlopt_cxx
             HINTS ${PC_NLOPT_LIBDIR} ${PC_NLOPT_LIBRARY_DIRS} )

set(NLOPT_LIBRARIES ${NLOPT_LIBRARY})
set(NLOPT_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NLOPT DEFAULT_MSG
                                  NLOPT_LIBRARY NLOPT_INCLUDE_DIR)

mark_as_advanced(NLOPT_INCLUDE_DIR NLOPT_LIBRARY)
