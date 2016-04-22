###############################################
# apps/atlasSimbicon
file(GLOB atlasSimbicon_srcs "*.cpp")
file(GLOB atlasSimbicon_hdrs "*.h")
add_executable(atlasSimbicon ${atlasSimbicon_srcs} ${atlasSimbicon_hdrs})
target_link_libraries(atlasSimbicon dart dart-gui dart-utils-urdf)
set_target_properties(atlasSimbicon PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
