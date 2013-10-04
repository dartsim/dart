# Find Eigen
#
# This sets the following variables:
# Eigen_FOUND
# Eigen_INCLUDE_DIRS

find_path(Eigen_INCLUDE_DIR
    NAMES Eigen/Core
    PATHS "${CMAKE_INSTALL_PREFIX}/include"
    PATH_SUFFIXES eigen3 eigen)

set(Eigen_INCLUDE_DIRS ${Eigen_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen DEFAULT_MSG Eigen_INCLUDE_DIR)

mark_as_advanced(Eigen_INCLUDE_DIR)