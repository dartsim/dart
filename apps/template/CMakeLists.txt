###############################################
# apps/CntctLrnin
file(GLOB CntctLrnin_srcs "*.cpp")
file(GLOB CntctLrnin_hdrs "*.h")
add_executable(CntctLrnin ${CntctLrnin_srcs} ${CntctLrnin_hdrs})
target_link_libraries(CntctLrnin dart)
set_target_properties(CntctLrnin PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
