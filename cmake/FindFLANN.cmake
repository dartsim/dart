# Find FLANN
#
# This sets the following variables:
# FLANN_FOUND
# FLANN_INCLUDE_DIRS

find_package(PkgConfig QUIET)

pkg_check_modules(PC_FLANN flann)
find_path(FLANN_INCLUDE_DIR
    NAMES flann/flann.h
    HINTS ${PC_FLANN_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

set(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FLANN DEFAULT_MSG FLANN_INCLUDE_DIR)

mark_as_advanced(FLANN_INCLUDE_DIR)
