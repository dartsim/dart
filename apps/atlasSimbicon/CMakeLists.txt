###############################################
# apps/atlasSimbicon
file(GLOB atlasSimbicon_srcs "*.cpp")
file(GLOB atlasSimbicon_hdrs "*.hpp")
add_executable(atlasSimbicon ${atlasSimbicon_srcs} ${atlasSimbicon_hdrs})
target_link_libraries(atlasSimbicon kido kido-gui)
set_target_properties(atlasSimbicon PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
