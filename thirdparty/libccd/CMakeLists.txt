cmake_minimum_required(VERSION 2.8)
project(libccd C)

set(CCD_VERSION "1.4")
set(CCD_SOVERSION "2")
set(PKG_DESC "Library for collision detection between convex shapes")
set(LIB_SUFFIX "" CACHE STRING  "Library installation directory suffix")

SET(CPACK_PACKAGE_VERSION_MAJOR ${CCD_VERSION})
SET(CPACK_PACKAGE_VERSION_MINOR ${CCD_SOVERSION})
SET(CPACK_PACKAGE_VERSION_PATCH "1") # maintained by Golems
SET(CPACK_PACKAGE_NAME "libccd")
SET(CPACK_SYSTEM_NAME "amd64")

include_directories("src/")

# make sure only DOUBLE or SINGLE is set; default to SINGLE
if (${CCD_SINGLE})
  set(CCD_DOUBLE false)
else()
  set(CCD_SINGLE true)
endif()

if (${CCD_DOUBLE})
  set(CCD_SINGLE false)
endif()

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/ccd/config.h.cmake.in" "${CMAKE_CURRENT_SOURCE_DIR}/src/ccd/config.h")

set(SOURCE_FILES
  src/ccd.c
  src/mpr.c
  src/polytope.c
  src/support.c
  src/vec3.c)

add_library(ccd SHARED ${SOURCE_FILES})
set_target_properties(ccd PROPERTIES VERSION ${FCL_VERSION} SOVERSION ${FCL_SOVERSION})

add_library(ccd_static STATIC ${SOURCE_FILES})
set_target_properties(ccd_static PROPERTIES OUTPUT_NAME ccd)

if(UNIX)
  find_library(MATH m)
  target_link_libraries(ccd ${MATH})  
  target_link_libraries(ccd_static ${MATH})
endif(UNIX)

set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/ccd.pc")
configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)

# old setup
# this setup was used when libccd was compiled everytime with main dart
# install(TARGETS ccd ccd_static ARCHIVE DESTINATION lib${LIB_SUFFIX}/ LIBRARY DESTINATION lib${LIB_SUFFIX}/)
# install(DIRECTORY src/ccd DESTINATION include/ccd FILES_MATCHING PATTERN "*.h")
# install(FILES "${pkg_conf_file}" DESTINATION lib${LIB_SUFFIX}/pkgconfig/ COMPONENT pkgconfig)

# new setup
# this setup when an external ccd debian is created
install(TARGETS ccd ccd_static DESTINATION lib)
install(DIRECTORY src/ccd DESTINATION include FILES_MATCHING PATTERN "*.h")
install(FILES "${pkg_conf_file}" DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)

SET(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Golems")

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${PKG_DESC})

INCLUDE(CPack)