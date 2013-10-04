###############################################
# apps/viewer

project(viewer)
file(GLOB viewer_srcs "*.cpp")
file(GLOB viewer_hdrs "*.h")
add_executable(viewer ${viewer_srcs} ${viewer_hdrs})
target_link_libraries(viewer dart ${DART_DEPENDENCIES})
set_target_properties(viewer PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
