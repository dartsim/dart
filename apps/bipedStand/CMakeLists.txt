###############################################
# apps/bipedStand
file(GLOB bipedStand_srcs "*.cpp")
file(GLOB bipedStand_hdrs "*.h")
add_executable(bipedStand ${bipedStand_srcs} ${bipedStand_hdrs})
target_link_libraries(bipedStand kido)
set_target_properties(bipedStand PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
