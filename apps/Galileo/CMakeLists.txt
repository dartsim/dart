###############################################
# apps/CS7496_project1

project(CS7496_project1)
file(GLOB CS7496_project1_srcs "*.cpp")
file(GLOB CS7496_project1_hdrs "*.h")
add_executable(CS7496_project1 ${CS7496_project1_srcs} ${CS7496_project1_hdrs})
target_link_libraries(CS7496_project1 dart ${DARTExt_LIBRARIES})
set_target_properties(CS7496_project1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
