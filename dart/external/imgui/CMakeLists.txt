dart_find_package(OpenGL)
dart_check_optional_package(OPENGL "dart-external-imgui" "OpenGL")

# Search all header and source files
file(GLOB hdrs "*.h")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-external-imgui)
set(component_name external-imgui)

dart_add_library(${target_name} ${hdrs} ${srcs})
target_include_directories(${target_name}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
target_link_libraries(${target_name} PUBLIC OpenGL::GL)
if(APPLE)
  target_link_libraries(${target_name} PUBLIC "-framework Cocoa")
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  target_compile_options(${target_name} PRIVATE -w)
endif()
target_compile_definitions(${target_name} PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})

# Install
install(
  FILES ${hdrs}
  DESTINATION include/dart/external/imgui
  COMPONENT headers
)
