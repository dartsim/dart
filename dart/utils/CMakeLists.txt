# Dependency checks
dart_find_package(tinyxml2)
dart_check_optional_package(TINYXML2 "dart-utils" "tinyxml2" "1.0.1")

# sdformat is optional when DART_ENABLE_SDFORMAT=OFF (e.g., Alt Linux)
if(DART_ENABLE_SDFORMAT)
  dart_find_package(sdformat)
  dart_check_optional_package(SDFORMAT "dart-utils" "sdformat" "16.0.0")
  if(NOT DART_HAVE_SDFORMAT)
    message(
      FATAL_ERROR
      "libsdformat (>= 16.0.0) is required for dart-utils. Please install it or "
      "set CMAKE_PREFIX_PATH so CMake can find sdformat-config.cmake. "
      "Alternatively, set DART_ENABLE_SDFORMAT=OFF to build without sdformat support."
    )
  endif()
  if(TARGET sdformat::sdformat)
    set_property(TARGET sdformat::sdformat PROPERTY IMPORTED_GLOBAL TRUE)
  else()
    message(
      FATAL_ERROR
      "libsdformat was found but the sdformat::sdformat imported target is "
      "missing. Please install the CMake config package that exports this "
      "target."
    )
  endif()
else()
  # Clear cached DART_HAVE_SDFORMAT to prevent stale values from previous
  # configurations where SDF was enabled
  set(DART_HAVE_SDFORMAT FALSE CACHE BOOL "Whether sdformat is available" FORCE)
endif()

find_package(CURL)
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
if(DART_ENABLE_SDFORMAT)
  add_subdirectory(sdf)
endif()

get_property(dart_utils_headers GLOBAL PROPERTY DART_UTILS_HEADERS)
get_property(dart_utils_sources GLOBAL PROPERTY DART_UTILS_SOURCES)

# Set local target name
set(target_name ${PROJECT_NAME}-utils)
set(component_name utils)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs} ${dart_utils_headers} ${dart_utils_sources})
target_link_libraries(${target_name} PUBLIC dart tinyxml2::tinyxml2)
if(DART_ENABLE_SDFORMAT)
  target_link_libraries(${target_name} PUBLIC sdformat::sdformat)
  target_compile_definitions(${target_name} PUBLIC DART_HAS_SDFORMAT=1)
else()
  target_compile_definitions(${target_name} PUBLIC DART_HAS_SDFORMAT=0)
endif()

if(HAVE_CURL)
  target_include_directories(${target_name} PRIVATE ${CURL_INCLUDE_DIRS})
  target_link_libraries(${target_name} PRIVATE CURL::libcurl)
  target_compile_definitions(${target_name} PUBLIC DART_HAS_CURL=1)
else()
  target_compile_definitions(${target_name} PUBLIC DART_HAS_CURL=0)
endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} tinyxml2)

# Generate header for this namespace
dart_get_filename_components(header_names "utils headers" ${hdrs})
if(urdfdom_FOUND)
  list(APPEND header_names "urdf/All.hpp")
endif()
set(header_names ${header_names} mjcf/All.hpp)
if(DART_ENABLE_SDFORMAT)
  list(APPEND header_names "sdf/All.hpp")
endif()
dart_generate_component_headers(
  COMPONENT_NAME utils
  TARGET_DIR "dart/utils/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs} ${detail_hdrs} ${dart_utils_headers}
)

# Install (includes All.hpp, utils.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/utils.hpp
  DESTINATION include/dart/utils
  COMPONENT headers
)
install(FILES ${detail_hdrs} DESTINATION include/dart/utils/detail COMPONENT headers)
# Install generated compatibility headers preserving subdirectory structure
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/utils
)

# Add subdirectories (components)
add_subdirectory(urdf)

dart_format_add(${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs} ${dart_utils_headers} ${dart_utils_sources})
