# Find TinyXML
#
# This sets the following variables:
# TinyXML_FOUND
# TinyXML_INCLUDE_DIRS
# TinyXML_LIBRARIES

find_path(TinyXML_INCLUDE_DIR tinyxml.h
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

set(TinyXML_INCLUDE_DIRS ${TinyXML_INCLUDE_DIR})

if(MSVC)
    set(TinyXML_LIBRARIES optimized tinyxml debug tinyxmld)
else()
    set(TinyXML_LIBRARIES tinyxml)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML DEFAULT_MSG TinyXML_INCLUDE_DIR)

mark_as_advanced(TinyXML_INCLUDE_DIR)