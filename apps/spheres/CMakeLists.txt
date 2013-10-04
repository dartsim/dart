###############################################
# apps/spheres
project(spheres)
file(GLOB spheres_srcs "*.cpp")
file(GLOB spheres_hdrs "*.h")
add_executable(spheres ${spheres_srcs} ${spheres_hdrs})
target_link_libraries(spheres dart ${DART_DEPENDENCIES})
set_target_properties(spheres PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
