###############################################
# apps/monkeybars
file(GLOB monkeybars_srcs "*.cpp")
file(GLOB monkeybars_hdrs "*.h")
add_executable(monkeybars ${monkeybars_srcs} ${monkeybars_hdrs})
target_link_libraries(monkeybars dart)
set_target_properties(monkeybars PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
