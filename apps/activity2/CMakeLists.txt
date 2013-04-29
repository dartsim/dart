###############################################
# apps/activity2

project(activity2)
file(GLOB activity2_srcs "*.cpp")
file(GLOB activity2_hdrs "*.h")
add_executable(activity2 ${activity2_srcs} ${activity2_hdrs})
target_link_libraries(activity2 dart ${DARTExt_LIBRARIES})
set_target_properties(activity2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
