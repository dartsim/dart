# Copyright (c) 2011-2021, The DART development contributors

file(GLOB_RECURSE sources "*.cpp")

set(target_name ${PROJECT_NAME}${DART_VERSION_MAJOR}-external-imgui)

# Check dependencies
if(NOT glfw3_FOUND)
  message(WARNING "Skipped [${target_name}] due to missing package glfw3")
  return()
endif()

add_library(${target_name} STATIC)
add_library(imgui::imgui ALIAS ${target_name})

target_sources(${target_name} PRIVATE ${sources})

target_include_directories(
  ${target_name}
  PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# TODO(JS): Check dependencies
target_link_libraries(${target_name} PUBLIC OpenGL::GL glfw ${CMAKE_DL_LIBS})
if(APPLE)
  target_link_libraries(${target_name} PUBLIC "-framework Cocoa")
endif()

if(MSVC)
  target_compile_options(${target_name} PRIVATE /W0)
else()
  target_compile_options(${target_name} PRIVATE -w -fPIC)
endif()

# Install targets
install(
  TARGETS ${target_name}
  EXPORT ${target_name}-targets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(
  EXPORT ${target_name}-targets
  FILE ${target_name}-targets.cmake
  DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}${DART_VERSION_MAJOR}
)
