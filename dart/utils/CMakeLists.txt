# Dependency checks
dart_find_package(tinyxml2)
if(NOT TINYXML2_FOUND OR DART_SKIP_TINYXML2)
  message(
    FATAL_ERROR
    "dart-io requires tinyxml2 >= 9.0.0; install it or set DART_BUILD_IO=OFF."
  )
endif()
dart_find_package(urdfdom)
if(NOT urdfdom_FOUND)
  message(
    FATAL_ERROR
    "dart-io requires urdfdom >= 4.0.0; install it or set DART_BUILD_IO=OFF."
  )
endif()
if(NOT DEFINED SDFORMAT_FOUND)
  dart_find_package(sdformat)
endif()
if(NOT SDFORMAT_FOUND)
  message(
    FATAL_ERROR
    "dart-io requires sdformat 14 (>= 14.8), 15 or 16; install it or set DART_BUILD_IO=OFF."
  )
endif()
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
add_subdirectory(urdf)

get_property(dart_utils_headers GLOBAL PROPERTY DART_UTILS_HEADERS)
get_property(dart_utils_sources GLOBAL PROPERTY DART_UTILS_SOURCES)

# Set local target name
set(target_name ${PROJECT_NAME}-io)
set(component_name io)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs} ${dart_utils_headers} ${dart_utils_sources})
target_link_libraries(
  ${target_name}
  PRIVATE ${SDFORMAT_LIBRARIES} urdfdom
  PUBLIC
    dart
    $<BUILD_INTERFACE:tinyxml2::tinyxml2>
    $<INSTALL_INTERFACE:$<1:tinyxml2::tinyxml2>>
)

# XmlHelpers.cpp uses fmt privately until the formatter dependency is removed.
if(TARGET fmt::fmt-header-only)
  target_link_libraries(
    ${target_name}
    PRIVATE "$<BUILD_INTERFACE:fmt::fmt-header-only>"
  )
else()
  target_compile_definitions(${target_name} PRIVATE FMT_HEADER_ONLY)
  target_include_directories(
    ${target_name}
    PRIVATE "$<TARGET_PROPERTY:fmt::fmt,INTERFACE_INCLUDE_DIRECTORIES>"
  )
endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} tinyxml2 sdformat)
if(NOT BUILD_SHARED_LIBS)
  add_component_dependency_packages(${PROJECT_NAME} ${component_name} urdfdom)
endif()

# Header sources retain legacy --target build commands without another library.
foreach(legacy_component utils utils-urdf)
  set(legacy_target ${PROJECT_NAME}-${legacy_component})
  add_library(${legacy_target} INTERFACE ${hdrs})
  target_link_libraries(${legacy_target} INTERFACE ${target_name})
  add_dependencies(${legacy_target} ${target_name})
  add_component(${PROJECT_NAME} ${legacy_component})
  add_component_targets(${PROJECT_NAME} ${legacy_component} ${legacy_target})
endforeach()
add_component_dependencies(${PROJECT_NAME} utils io)
add_component_dependencies(${PROJECT_NAME} utils-urdf utils)

# Generate header for this namespace
dart_get_filename_components(header_names "utils headers" ${hdrs})
set(header_names ${header_names} mjcf/mjcf.hpp sdf/sdf.hpp urdf/urdf.hpp)
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/utils.hpp"
  "dart/utils/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/utils.hpp
  DESTINATION include/dart/utils
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/utils/detail
  COMPONENT headers
)

if(DART_BUILD_UTILS_ASSETS)
  add_subdirectory(assets)
endif()

dart_format_add(${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs} ${dart_utils_headers} ${dart_utils_sources})
