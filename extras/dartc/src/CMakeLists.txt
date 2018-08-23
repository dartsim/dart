file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

add_library(dartc STATIC ${hdrs} ${srcs})
target_link_libraries(dartc PUBLIC dart)

set_property(TARGET dartc PROPERTY C_STANDARD 11)

dart_format_add(${hdrs} ${srcs})
