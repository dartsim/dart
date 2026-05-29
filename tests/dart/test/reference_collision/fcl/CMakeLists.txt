# Copyright (c) 2011, The DART development contributors

file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_format_add(${hdrs} ${srcs})

set(target_name ${PROJECT_NAME}-test-reference-fcl)
dart_add_library(${target_name} ${srcs})
target_include_directories(
  ${target_name}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/tests>
)
target_link_libraries(${target_name} PUBLIC dart fcl)
if(TARGET ${PROJECT_NAME}-collision-native)
  target_sources(
    ${target_name}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../dart_query_helper.cpp
  )
  target_link_libraries(${target_name} PUBLIC ${PROJECT_NAME}-collision-native)
  target_compile_definitions(${target_name} PRIVATE DART_FCL_HAS_DART=1)
endif()
target_compile_definitions(${target_name} PRIVATE DART_BUILDING_COLLISION_FCL)
