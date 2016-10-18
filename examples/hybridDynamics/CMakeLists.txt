###############################################################
# This file can be used as-is in the directory of any example,#
# however you might need to specify your own dependencies in  #
# target_link_libraries if your app depends on more than dart #
###############################################################
get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)
file(GLOB ${example_name}_srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${${example_name}_srcs})
dart_add_example(${example_name})
target_link_libraries(${example_name} dart dart-gui)
set_target_properties(${example_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

