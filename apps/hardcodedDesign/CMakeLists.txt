###############################################
# apps/skeleton

project(skeleton)
file(GLOB skeleton_srcs "*.cpp")
file(GLOB skeleton_hdrs "*.h")
add_executable(skeleton ${skeleton_srcs} ${skeleton_hdrs})
target_link_libraries(skeleton dart ${DARTExt_LIBRARIES})
set_target_properties(skeleton PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
