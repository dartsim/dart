# The legacy external-imgui component reaches gui as a dependency.
find_package(DART CONFIG REQUIRED COMPONENTS external-imgui)
include_directories(${DART_INCLUDE_DIRS})
add_executable(gui_legacy_alias_consumer ../legacy/main.cpp)
target_link_libraries(gui_legacy_alias_consumer PRIVATE ${DART_LIBRARIES})
add_test(NAME gui_legacy_alias_consumer COMMAND gui_legacy_alias_consumer)
