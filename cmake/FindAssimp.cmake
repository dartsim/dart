# Find Assimp
#
# This sets the following variables:
# Assimp_FOUND
# Assimp_INCLUDE_DIRS
# Assimp_LIBRARIES

find_path(Assimp_INCLUDE_DIR assimp/scene.h
    PATHS "${CMAKE_INSTALL_PREFIX}/include")
set(Assimp_LIBRARIES assimp)

set(Assimp_INCLUDE_DIRS ${Assimp_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Assimp DEFAULT_MSG Assimp_INCLUDE_DIR)

mark_as_advanced(Assimp_INCLUDE_DIR)