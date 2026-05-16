# Copyright (c) 2011, The DART development contributors

file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
dart_format_add(${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})

set(target_name ${PROJECT_NAME}-test-reference-ode)
dart_add_library(${target_name} ${srcs} ${detail_srcs})
target_include_directories(${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/tests>
)
if(ODE_INCLUDE_DIRS)
  target_include_directories(${target_name} PUBLIC ${ODE_INCLUDE_DIRS})
endif()
target_link_libraries(${target_name}
  PUBLIC
    dart
    ${_dart_ode_public_link}
)
if(TARGET ${PROJECT_NAME}-collision-native)
  target_link_libraries(${target_name} PUBLIC ${PROJECT_NAME}-collision-native)
  target_compile_definitions(${target_name} PRIVATE DART_ODE_HAS_DART=1)
endif()
target_compile_definitions(${target_name}
  PRIVATE
    DART_BUILDING_COLLISION_ODE
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${target_name} PRIVATE -Wno-deprecated-enum-enum-conversion)
endif()
