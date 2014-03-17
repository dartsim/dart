# Find SNOPT
#
# This sets the following variables:
# SNOPT_FOUND
# SNOPT_INCLUDE_DIRS
# SNOPT_LIBRARIES
# SNOPT_DEFINITIONS

find_package(PkgConfig QUIET)
pkg_check_modules(PC_SNOPT snopt)
set(SNOPT_DEFINITIONS ${PC_SNOPT_CFLAGS_OTHER})

find_path(SNOPT_INCLUDE_DIR cppsrc/snopt.hh
    HINTS ${PC_SNOPT_INCLUDEDIR} ${PC_SNOPT_INCLUDE_DIRS}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

find_library(SNOPT_LIBRARY NAMES snopt
             HINTS ${PC_SNOPT_LIBDIR} ${PC_SNOPT_LIBRARY_DIRS} )

set(SNOPT_LIBRARIES ${SNOPT_LIBRARY})
set(SNOPT_INCLUDE_DIRS ${SNOPT_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SNOPT DEFAULT_MSG
                                  SNOPT_LIBRARY SNOPT_INCLUDE_DIR)

mark_as_advanced(SNOPT_INCLUDE_DIR SNOPT_LIBRARY)
