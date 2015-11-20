# Enable multi-threaded compilation.
# We do this here and not in the root folder since the example apps
# do not have enough source files to benefit from this.
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Add subdirectories
add_subdirectory(collision)
add_subdirectory(common)
add_subdirectory(constraint)
add_subdirectory(dynamics)
add_subdirectory(optimizer)
add_subdirectory(integration)
add_subdirectory(lcpsolver)
add_subdirectory(math)
add_subdirectory(renderer)
add_subdirectory(simulation)
if(NOT BUILD_CORE_ONLY)
  add_subdirectory(gui)
  add_subdirectory(planning)
  add_subdirectory(utils)
endif()

# Set header and source files
set(dart_core_hdrs
  dart-core.h
  ${dart_common_hdrs}
  ${dart_math_hdrs}
  ${dart_integration_hdrs}
  ${dart_lcpsolver_hdrs}
  ${dart_dynamics_hdrs}
  ${dart_optimizer_hdrs}
  ${dart_collision_hdrs}
  ${dart_constraint_hdrs}
  ${dart_renderer_hdrs}
  ${dart_simulation_hdrs}
)
set(dart_core_srcs
  ${dart_common_srcs}
  ${dart_math_srcs}
  ${dart_integration_srcs}
  ${dart_lcpsolver_srcs}
  ${dart_dynamics_srcs}
  ${dart_optimizer_srcs}
  ${dart_collision_srcs}
  ${dart_constraint_srcs}
  ${dart_renderer_srcs}
  ${dart_simulation_srcs}
)
if(NOT BUILD_CORE_ONLY)
  set(dart_hdrs
    dart.h
    ${dart_gui_hdrs}
    ${dart_planning_hdrs}
    ${dart_utils_hdrs}
  )
  set(dart_srcs
    ${dart_gui_srcs}
    ${dart_planning_srcs}
    ${dart_utils_srcs}
  )
endif()

# Library: dart-core
dart_add_library(dart-core ${dart_core_hdrs} ${dart_core_srcs})
target_link_libraries(dart-core ${DART_CORE_DEPENDENCIES})

# Library: dart
if(NOT BUILD_CORE_ONLY)
  dart_add_library(dart ${dart_hdrs} ${dart_srcs})
  target_link_libraries(dart dart-core ${DART_DEPENDENCIES})
endif()

if(MSVC)
  set_target_properties(
    ${target} PROPERTIES
    STATIC_LIBRARY_FLAGS_RELEASE "/LTCG"
  )
endif()

#install(FILES ${dart_core_hdrs} DESTINATION include/dart/ COMPONENT headers)
install(FILES dart-core.h DESTINATION include/dart/ COMPONENT headers)
install(TARGETS dart-core EXPORT DARTCoreTargets DESTINATION lib)
if(NOT BUILD_CORE_ONLY)
#  install(FILES ${dart_hdrs} DESTINATION include/dart/ COMPONENT headers)
  install(FILES dart.h DESTINATION include/dart/ COMPONENT headers)
  install(TARGETS dart-core dart EXPORT DARTTargets DESTINATION lib)
endif()

