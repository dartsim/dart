###############################################
# apps/bipedStand
file(GLOB bipedStand_srcs "*.cpp")
file(GLOB bipedStand_hdrs "*.hpp")
add_executable(bipedStand ${bipedStand_srcs} ${bipedStand_hdrs})
target_link_libraries(bipedStand dart dart-gui)
set_target_properties(bipedStand PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
