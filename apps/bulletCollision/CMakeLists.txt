###############################################
# apps/bulletCollision
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")
add_executable(bulletCollision ${srcs} ${hdrs})
target_link_libraries(bulletCollision dart)
set_target_properties(bulletCollision PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
