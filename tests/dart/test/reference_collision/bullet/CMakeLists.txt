# Copyright (c) 2011, The DART development contributors

file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
dart_format_add(${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})

set(_dart_bullet_link_items ${BULLET_LIBRARIES})
if(TARGET Bullet::Bullet)
  list(APPEND _dart_bullet_link_items Bullet::Bullet)
endif()
if(NOT _dart_bullet_link_items)
  set(_bullet_candidate_libs
    ${BULLET_COLLISION_LIBRARY}
    ${BULLET_DYNAMICS_LIBRARY}
    ${BULLET_SOFTBODY_LIBRARY}
    ${BULLET_MATH_LIBRARY}
  )
  foreach(_bullet_candidate IN LISTS _bullet_candidate_libs)
    if(_bullet_candidate)
      list(APPEND _dart_bullet_link_items ${_bullet_candidate})
    endif()
  endforeach()
endif()
list(REMOVE_DUPLICATES _dart_bullet_link_items)
if(NOT _dart_bullet_link_items)
  message(
    FATAL_ERROR
    "Bullet was found but did not report any linkable libraries."
  )
endif()

set(target_name ${PROJECT_NAME}-test-reference-bullet)
dart_add_library(${target_name} ${srcs} ${detail_srcs})
target_include_directories(${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/tests>
)
if(BULLET_INCLUDE_DIRS)
  target_include_directories(${target_name} SYSTEM PUBLIC ${BULLET_INCLUDE_DIRS})
endif()
target_link_libraries(${target_name}
  PUBLIC
    dart
    ${_dart_bullet_link_items}
)
if(TARGET ${PROJECT_NAME}-collision-native)
  target_link_libraries(${target_name} PUBLIC ${PROJECT_NAME}-collision-native)
  target_compile_definitions(${target_name} PRIVATE DART_BULLET_HAS_DART=1)
endif()
target_compile_definitions(${target_name}
  PRIVATE
    DART_BUILDING_COLLISION_BULLET
)
if(BT_USE_DOUBLE_PRECISION)
  target_compile_definitions(${target_name} PUBLIC BT_USE_DOUBLE_PRECISION)
endif()
