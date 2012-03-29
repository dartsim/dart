project(libccd C)

set(CCD_VERSION "1.2")
set(PKG_DESC "Library for collision detection between convex shapes")

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

find_library(MATH m)

set(SOURCE_FILES
  src/alloc.c
  src/ccd.c
  src/mpr.c
  src/polytope.c
  src/support.c
  src/vec3.c)

add_library(ccd SHARED ${SOURCE_FILES})

target_link_libraries(ccd ${MATH})

add_library(ccd_static STATIC ${SOURCE_FILES})
set_target_properties(ccd_static PROPERTIES OUTPUT_NAME ccd)
target_link_libraries(ccd_static ${MATH})

set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/ccd.pc")
configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)

install(TARGETS ccd ccd_static ARCHIVE DESTINATION lib/ LIBRARY DESTINATION lib/)
install(DIRECTORY src/ccd DESTINATION include FILES_MATCHING PATTERN "*.h")
install(FILES "${pkg_conf_file}" DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
