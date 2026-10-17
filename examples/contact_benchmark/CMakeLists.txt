cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(required_components collision-bullet collision-ode utils)
set(required_libraries dart dart-collision-bullet dart-collision-ode dart-utils)

# The headless benchmark needs no GUI; --gui and --gui-capture need dart-gui.
if(DART_IN_SOURCE_BUILD)
  set(has_gui 0)
  if(TARGET dart-gui)
    list(APPEND required_libraries dart-gui)
    set(has_gui 1)
  endif()
  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
  if(TARGET ${example_name})
    target_link_libraries(${example_name} ${SDFORMAT_LIBRARIES})
    target_compile_definitions(
      ${example_name}
      PRIVATE DART_CONTACT_BENCHMARK_HAS_GUI=${has_gui}
    )
  endif()
  return()
endif()

find_package(
  DART
  6.19
  REQUIRED
  COMPONENTS ${required_components}
  OPTIONAL_COMPONENTS gui
  CONFIG
)
file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
target_link_libraries(${example_name} PRIVATE ${SDFORMAT_LIBRARIES})
include("${CMAKE_CURRENT_LIST_DIR}/../CheckFilamentGui.cmake")
if(DART_EXAMPLES_HAVE_FILAMENT_GUI)
  target_link_libraries(${example_name} PUBLIC dart-gui)
  target_compile_definitions(
    ${example_name}
    PRIVATE DART_CONTACT_BENCHMARK_HAS_GUI=1
  )
else()
  message(
    STATUS
    "Building contact_benchmark without GUI modes: the Filament gui component (dart/gui/Viewer.hpp) is unavailable"
  )
  target_compile_definitions(
    ${example_name}
    PRIVATE DART_CONTACT_BENCHMARK_HAS_GUI=0
  )
endif()
