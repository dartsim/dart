# Get the name of this directory
get_filename_component(example_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)

# Compile the example
add_executable(${example_name} SimulationEventHandler.cpp SimulationEventHandler.hpp main.cpp)
target_link_libraries(${example_name} dart dart-gui-osg)
dart_format_add(${example_name})
