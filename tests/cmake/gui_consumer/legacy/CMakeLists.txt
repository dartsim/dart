include_directories(${DART_INCLUDE_DIRS})
add_executable(gui_legacy_consumer main.cpp)
target_link_libraries(gui_legacy_consumer PRIVATE ${DART_LIBRARIES})
add_test(NAME gui_legacy_consumer COMMAND gui_legacy_consumer)
