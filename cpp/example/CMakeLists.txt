# Copyright (c) 2011-2022, The DART development contributors

add_subdirectory(simple_simulator)

# Add custom target to build all the examples as a single target
dart_property_get(DART_GLOBAL_PROPERTY_EXAMPLE_LIST example_targets)
add_custom_target(examples DEPENDS ${example_targets})
