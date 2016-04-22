###############################################
# apps/softDropBoxTest
set(APP_NAME softBodies)
file(GLOB ${APP_NAME}_srcs "*.cpp")
file(GLOB ${APP_NAME}_hdrs "*.h")
add_executable(${APP_NAME} ${${APP_NAME}_srcs} ${${APP_NAME}_hdrs})
target_link_libraries(${APP_NAME} dart dart-gui)
set_target_properties(${APP_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
