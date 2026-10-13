cmake_minimum_required(VERSION 3.22.1)

project(dart-examples)

if(DART_IN_SOURCE_BUILD)
  dart_format_add(GuiOptions.hpp)
endif()

# Non-GUI examples
add_subdirectory(cylindrical_constraint)
add_subdirectory(speed_test)
# Headless benchmark; its --gui modes need DART_BUILD_GUI=ON.
add_subdirectory(contact_benchmark)

# GUI examples
if(NOT DART_IN_SOURCE_BUILD)
  find_package(DART QUIET OPTIONAL_COMPONENTS gui CONFIG)
endif()
include("${CMAKE_CURRENT_LIST_DIR}/CheckFilamentGui.cmake")
if(DART_IN_SOURCE_BUILD OR DART_EXAMPLES_HAVE_FILAMENT_GUI)
  add_subdirectory(hello_world)
else()
  message(
    STATUS
    "Skipping GUI examples: they need the Filament gui component (dart/gui/Viewer.hpp)"
  )
endif()
