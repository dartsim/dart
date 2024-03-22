# Set local target name
set(target_name ${PROJECT_NAME}-gui-rerun)
set(component_name gui-rerun)

# Check dart component dependencies
dart_check_dependent_target(${target_name} dart-gui)

# Check external dependencies
if(NOT TARGET rerun_sdk)
  message(FATAL_ERROR "Failed to set up rerun_sdk target when DART_BUILD_RERUN=${DART_BUILD_RERUN}")
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

set(dart_gui_rerun_hdrs ${hdrs} ${detail_hdrs})
set(dart_gui_rerun_srcs ${srcs} ${detail_srcs})

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${dart_gui_rerun_hdrs} ${dart_gui_rerun_srcs})
target_link_libraries(${target_name} PUBLIC dart-gui rerun_sdk)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} gui)
# add_component_dependency_packages(${PROJECT_NAME} ${component_name} rerun_sdk)

# Generate header for this namespace
dart_get_filename_components(header_names "gui rerun headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/rerun.hpp"
  "dart/gui/rerun/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/rerun.hpp
  DESTINATION include/dart/gui/rerun
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/gui/rerun/detail
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs} ${dart_gui_osg_hdrs} ${dart_gui_osg_srcs})
