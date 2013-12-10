# Find FCL
#
# This sets the following variables:
# FCL_FOUND
# FCL_INCLUDE_DIRS
# FCL_LIBRARIES

find_package(PkgConfig QUIET)

pkg_check_modules(PC_FCL fcl)
find_path(FCL_INCLUDE_DIR fcl/collision.h
    HINTS ${PC_FCL_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

set(FCL_INCLUDE_DIRS ${FCL_INCLUDE_DIR})

if(MSVC)
    set(FCL_LIBRARIES optimized fcl debug fcld ccd)
else()
    find_library(FCL_LIBRARY fcl)
    find_library(CCD_LIBRARY ccd)
    set(FCL_LIBRARIES ${FCL_LIBRARY} ${CCD_LIBRARY})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FCL DEFAULT_MSG FCL_INCLUDE_DIR)

mark_as_advanced(FCL_INCLUDE_DIR FCL_LIBRARY CCD_LIBRARY)