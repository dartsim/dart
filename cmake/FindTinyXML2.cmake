# Find TinyXML
#
# This sets the following variables:
# TinyXML2_FOUND
# TinyXML2_INCLUDE_DIRS
# TinyXML2_LIBRARIES

find_path(TinyXML2_INCLUDE_DIR tinyxml2.h
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

set(TinyXML2_INCLUDE_DIRS ${TinyXML2_INCLUDE_DIR})

if(MSVC)
    set(TinyXML2_LIBRARIES optimized tinyxml2 debug tinyxml2d)
else()
    set(TinyXML2_LIBRARIES tinyxml2)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML2 DEFAULT_MSG TinyXML2_INCLUDE_DIR)

mark_as_advanced(TinyXML2_INCLUDE_DIR)