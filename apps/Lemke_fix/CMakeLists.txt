###############################################
# apps/Lemke_Fix
file(GLOB Lemke_Fix_srcs "*.cpp")
file(GLOB Lemke_Fix_hdrs "*.h")
add_executable(Lemke_Fix ${Lemke_Fix_srcs} ${Lemke_Fix_hdrs})
target_link_libraries(Lemke_Fix dart)
set_target_properties(Lemke_Fix PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
