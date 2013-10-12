###############################################
# apps/hardcodedDesign

file(GLOB hardcodedDesign_srcs "*.cpp")
file(GLOB hardcodedDesign_hdrs "*.h")
add_executable(hardcodedDesign ${hardcodedDesign_srcs} ${hardcodedDesign_hdrs})
target_link_libraries(hardcodedDesign dart)
set_target_properties(hardcodedDesign PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
