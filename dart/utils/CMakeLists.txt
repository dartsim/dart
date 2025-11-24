if(NOT DEFINED TINYXML2_FOUND)
  set(TINYXML2_FOUND FALSE)
endif()
if(NOT DEFINED SDFORMAT_FOUND)
  set(SDFORMAT_FOUND FALSE)
endif()
if(NOT DEFINED CURL_FOUND)
  set(CURL_FOUND FALSE)
endif()

dart_check_optional_package(TINYXML2 "dart-utils" "tinyxml2" "1.0.1")

dart_check_optional_package(SDFORMAT "dart-utils" "sdformat" "16.0.0")
if(NOT HAVE_SDFORMAT)
  message(
    FATAL_ERROR
    "libsdformat (>= 16.0.0) is required for dart-utils. Please install it or "
    "set CMAKE_PREFIX_PATH so CMake can find sdformat-config.cmake.")
  message(WARNING "Skipping dart-utils: required dependency sdformat missing.")
  return()
endif()
if(NOT HAVE_TINYXML2)
  message(WARNING "Skipping dart-utils: required dependency tinyxml2 missing.")
  return()
endif()
if(TARGET sdformat::sdformat)
  set_property(TARGET sdformat::sdformat PROPERTY IMPORTED_GLOBAL TRUE)
else()
  message(
    FATAL_ERROR
    "libsdformat was found but the sdformat::sdformat imported target is "
    "missing. Please install the CMake config package that exports this "
    "target.")
endif()

dart_check_optional_package(CURL "dart-utils HTTP retriever" "libcurl")

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

function(dart_add_utils_headers)
  dart_property_add(DART_UTILS_HEADERS ${ARGN})
endfunction(dart_add_utils_headers)

function(dart_add_utils_sources)
  dart_property_add(DART_UTILS_SOURCES ${ARGN})
endfunction(dart_add_utils_sources)

# Add subdirectories
add_subdirectory(mjcf)
add_subdirectory(sdf)

get_property(dart_utils_headers GLOBAL PROPERTY DART_UTILS_HEADERS)
get_property(dart_utils_sources GLOBAL PROPERTY DART_UTILS_SOURCES)

# Set local target name
set(target_name ${PROJECT_NAME}-utils)
set(component_header_extras
  mjcf/All.hpp
  sdf/All.hpp
)
if(urdfdom_FOUND)
  list(APPEND component_header_extras urdf/All.hpp)
endif()

set(_dart_utils_private_includes)
set(_dart_utils_private_links)
set(_dart_utils_public_defs)

if(HAVE_CURL)
  list(APPEND _dart_utils_private_includes ${CURL_INCLUDE_DIRS})
  list(APPEND _dart_utils_private_links CURL::libcurl)
  list(APPEND _dart_utils_public_defs DART_HAS_CURL=1)
else()
  list(APPEND _dart_utils_public_defs DART_HAS_CURL=0)
endif()

# Add target
dart_add_library(
  NAME ${target_name}
  HEADERS ${hdrs} ${detail_hdrs} ${dart_utils_headers}
  SOURCES ${srcs} ${detail_srcs} ${dart_utils_sources}
  PUBLIC_LINK_LIBRARIES dart tinyxml2::tinyxml2 sdformat::sdformat
  PRIVATE_LINK_LIBRARIES ${_dart_utils_private_links}
  PRIVATE_INCLUDE_DIRS ${_dart_utils_private_includes}
  PUBLIC_COMPILE_DEFINITIONS ${_dart_utils_public_defs}
)

# Component
dart_add_component(
  NAME utils
  PACKAGE ${PROJECT_NAME}
  TARGETS ${target_name}
  DEPENDENCIES dart
  DEPENDENCY_PACKAGES tinyxml2 sdformat
  COMPONENT_HEADERS ${hdrs}
  COMPONENT_HEADER_EXTRAS ${component_header_extras}
  COMPONENT_HEADER_SOURCES ${hdrs} ${detail_hdrs} ${dart_utils_headers}
  COMPONENT_TARGET_DIR "dart/utils/"
  COMPONENT_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
)

# Add subdirectories (components)
add_subdirectory(urdf)
