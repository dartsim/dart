find_package(filament QUIET)
dart_check_optional_package(filament "dart-gui-flmt" "filament" "0.0.0")

find_package(SDL2 CONFIG QUIET)
if(NOT SDL2_FOUND)
  find_package(SDL2 MODULE QUIET)
endif()
dart_check_optional_package(SDL2 "dart-gui-flmt" "sdl2")

find_package(Threads)
dart_check_optional_package(Threads "dart-gui-flmt" "threads")

# Compiler check
if("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU"
    OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  message(WARNING
    "The GCC compiler is not supported for dart-gui-flmt, please use Clang for "
    "both C and C++. Skipping building dart-gui-flmt.")
  return()
endif()
if(CLANG_VERSION VERSION_LESS 3.4)
  message(FATAL_ERROR "The installed Clang version is ${CLANG_VERSION}. "
      "dart-gui-flmt requires clang 3.3 or greater."
  )
  return()
endif()
if(UNIX AND NOT APPLE AND NOT ANDROID AND NOT WEBGL)
  set(LINUX TRUE)
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp" "*.h")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
if (WIN32)
  list(APPEND srcs linux/NativeWindowHelperWindows.cpp)
  list(APPEND srcs win32/Path.cpp)
endif()
if (LINUX OR ANDROID)
  list(APPEND srcs linux/NativeWindowHelperLinux.cpp)
  list(APPEND srcs linux/Path.cpp)
endif()
if (APPLE)
  list(APPEND srcs darwin/NativeWindowHelperCocoa.mm)
  list(APPEND APP_LIBS "-framework Cocoa")
  list(APPEND srcs darwin/Path.cpp)
endif()

# Set local target name
set(target_name ${PROJECT_NAME}-gui-flmt)
set(component_name gui-flmt)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC
    ${filament_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  PUBLIC
    dart
    dart-gui
    ${filament_LIBRARIES}
    ${SDL2_LIBRARIES}
    ${APP_LIBS}
)
# TODO(JS): Check if C++14 is supported
target_compile_options(${target_name} PUBLIC -std=c++14)

# Thread
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(${target_name} PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(${target_name} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} gui)

# Generate header for this namespace
dart_get_filename_components(header_names "gui_filament_headers headers" ${hdrs})
dart_generate_include_header_list(
  gui_filament_headers
  "dart/gui/filament/"
  "gui_filament_headers headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/filament.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/filament.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/filament.hpp
  DESTINATION include/dart/gui/filament
  COMPONENT headers
)

dart_format_add(${hdrs} ${detail_hdrs} ${srcs} ${detail_srcs})
