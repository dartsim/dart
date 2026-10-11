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
if(
  DART_IN_SOURCE_BUILD
  OR (DART_gui_FOUND AND DART_VERSION VERSION_GREATER_EQUAL 7.0)
)
  add_subdirectory(hello_world)
  add_subdirectory(demos)
else()
  message(
    STATUS
    "Skipping GUI examples: they need DART 7 with its gui component"
  )
endif()
