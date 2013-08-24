###############################################
# apps/elevator

project(elevator)
file(GLOB elevator_srcs "*.cpp")
file(GLOB elevator_hdrs "*.h")
add_executable(elevator ${elevator_srcs} ${elevator_hdrs})
target_link_libraries(elevator dart ${DARTExt_LIBRARIES})
set_target_properties(elevator PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
