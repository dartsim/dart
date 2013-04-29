###############################################
# apps/activity1

project(activity1)
file(GLOB activity1_srcs "*.cpp")
file(GLOB activity1_hdrs "*.h")
add_executable(activity1 ${activity1_srcs} ${activity1_hdrs})
target_link_libraries(activity1 dart ${DARTExt_LIBRARIES})
set_target_properties(activity1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
