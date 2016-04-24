# Enable multi-threaded compilation.
# We do this here and not in the root folder since the examples and the
# tutorials do not have enough source files to benefit from this.
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
add_subdirectory(simulation)

# Set header and source files
get_filename_component(dart_h "dart.h" ABSOLUTE)
set(dart_hdrs
  ${dart_h}
  ${dart_common_hdrs}
  ${dart_math_hdrs}
  ${dart_integration_hdrs}
  ${dart_lcpsolver_hdrs}
  ${dart_dynamics_hdrs}
  ${dart_optimizer_hdrs}
  ${dart_collision_hdrs}
  ${dart_constraint_hdrs}
  ${dart_simulation_hdrs}
)
set(dart_srcs
  ${dart_common_srcs}
  ${dart_math_srcs}
  ${dart_integration_srcs}
  ${dart_lcpsolver_srcs}
  ${dart_dynamics_srcs}
  ${dart_optimizer_srcs}
  ${dart_collision_srcs}
  ${dart_constraint_srcs}
  ${dart_simulation_srcs}
)

# Library: dart
dart_add_library(dart ${dart_hdrs} ${dart_srcs})
target_link_libraries(dart ${CCD_LIBRARIES}
                           ${FCL_LIBRARIES}
                           ${ASSIMP_LIBRARIES}
                           ${Boost_LIBRARIES})
if(HAVE_BULLET_COLLISION)
  target_link_libraries(dart ${BULLET_LIBRARIES})
endif()

if(MSVC)
  set_target_properties(
    ${target} PROPERTIES
    STATIC_LIBRARY_FLAGS_RELEASE "/LTCG"
  )
endif()

install(FILES dart.h DESTINATION include/dart/ COMPONENT headers)
install(TARGETS dart EXPORT DARTTargets DESTINATION lib)

# Coverage test files
set(dart_coveralls_files "${dart_coveralls_files};${dart_hdrs};${dart_srcs}"
    CACHE INTERNAL "Files for coverage tests" FORCE)

if(FLANN_FOUND)
  add_subdirectory(planning)
endif()

if(OPENGL_FOUND AND GLUT_FOUND)
  add_subdirectory(gui)
endif()

if(TINYXML_FOUND AND TINYXML2_FOUND AND urdfdom_FOUND)
  add_subdirectory(utils)
endif()

# Setup the coveralls target and tell it to gather
# coverage data for all the lib sources.
if(DART_COVERALLS)
  coveralls_setup("${dart_coveralls_files}" ${DART_COVERALLS_UPLOAD})
endif()
