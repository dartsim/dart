# Find IPOPT
#
# This sets the following variables:
# IPOPT_FOUND
# IPOPT_INCLUDE_DIRS
# IPOPT_LIBRARIES
# IPOPT_DEFINITIONS

find_package(PkgConfig QUIET)
pkg_check_modules(PC_IPOPT ipopt QUIET)
set(IPOPT_DEFINITIONS ${PC_IPOPT_CFLAGS_OTHER})

find_path(IPOPT_INCLUDE_DIR IpIpoptNLP.hpp
    HINTS ${PC_IPOPT_INCLUDEDIR} ${PC_IPOPT_INCLUDE_DIRS}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

find_library(IPOPT_LIBRARY NAMES ipopt
             HINTS ${PC_IPOPT_LIBDIR} ${PC_IPOPT_LIBRARY_DIRS} )

set(IPOPT_LIBRARIES ${IPOPT_LIBRARY})
set(IPOPT_INCLUDE_DIRS ${IPOPT_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(IPOPT DEFAULT_MSG
                                  IPOPT_LIBRARY IPOPT_INCLUDE_DIR)

mark_as_advanced(IPOPT_INCLUDE_DIR IPOPT_LIBRARY)
