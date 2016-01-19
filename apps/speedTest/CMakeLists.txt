###############################################################
# This file can be used as-is in the directory of any app,    #
# however you might need to specify your own dependencies in  #
# target_link_libraries if your app depends on more than kido #
###############################################################
get_filename_component(app_name ${CMAKE_CURRENT_LIST_DIR} NAME)
file(GLOB ${app_name}_srcs "*.cpp" "*.hpp" "*.hpp")
add_executable(${app_name} ${${app_name}_srcs})
target_link_libraries(${app_name} kido kido-gui)
set_target_properties(${app_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
