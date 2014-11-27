# Enable multi-threaded compilation.
# We do this here and not in the root folder since the example apps
# do not have enough source files to benefit from this.
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Set header and source files
set(corehdrs dart-core.h)
set(hdrs dart.h ${corehdrs})
set(srcs dart.cpp)

# Add subdirectories
add_subdirectory(collision)
add_subdirectory(common)
add_subdirectory(constraint)
add_subdirectory(dynamics)
add_subdirectory(integration)
add_subdirectory(lcpsolver)
add_subdirectory(math)
add_subdirectory(renderer)
add_subdirectory(simulation)
if(NOT BUILD_CORE_ONLY)
  add_subdirectory(gui)
  add_subdirectory(optimizer)
  add_subdirectory(planning)
  add_subdirectory(utils)
endif()

# Library: dart-core
dart_add_library(dart-core ${corehdrs} ${srcs})
target_link_libraries(
  dart-core
  dart_common
  dart_math
  dart_integration
  dart_lcpsolver
  dart_dynamics
  dart_collision
  dart_constraint
  dart_simulation
  ${DART_CORE_DEPENDENCIES}
)

# Library: dart
if(NOT BUILD_CORE_ONLY)
  dart_add_library(dart ${hdrs} ${srcs})
  target_link_libraries(
    dart
    dart-core
    dart_gui
    dart_optimizer
    dart_planning
    dart_renderer
    dart_utils
    ${DART_DEPENDENCIES}
  )
endif()

if(MSVC)
  set_target_properties(
    ${target} PROPERTIES
    STATIC_LIBRARY_FLAGS_RELEASE "/LTCG"
  )
endif()

install(FILES ${corehdrs} DESTINATION include/dart/ COMPONENT headers)
install(TARGETS dart-core EXPORT DARTCoreTargets DESTINATION lib)
if(NOT BUILD_CORE_ONLY)
  install(FILES ${hdrs} DESTINATION include/dart/ COMPONENT headers)
  install(TARGETS dart-core dart EXPORT DARTTargets DESTINATION lib)
endif()

