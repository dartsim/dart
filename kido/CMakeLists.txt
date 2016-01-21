# Enable multi-threaded compilation.
# We do this here and not in the root folder since the example apps
# do not have enough source files to benefit from this.
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Add subdirectories
add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(integration)
add_subdirectory(collision)
add_subdirectory(lcpsolver)
add_subdirectory(constraint)
add_subdirectory(optimizer)
add_subdirectory(dynamics)
add_subdirectory(renderer)
add_subdirectory(simulation)

if(FLANN_FOUND)
  add_subdirectory(planning)
endif()

if(GLUT_FOUND)
  add_subdirectory(gui)
endif()

if(TINYXML_FOUND AND TINYXML2_FOUND AND urdfdom_FOUND)
  add_subdirectory(utils)
endif()

# Set header and source files
set(kido_hdrs
  kido.hpp
  ${kido_common_hdrs}
  ${kido_math_hdrs}
  ${kido_integration_hdrs}
  ${kido_lcpsolver_hdrs}
  ${kido_dynamics_hdrs}
  ${kido_optimizer_hdrs}
  ${kido_collision_hdrs}
  ${kido_constraint_hdrs}
  ${kido_renderer_hdrs}
  ${kido_simulation_hdrs}
)
set(kido_srcs
  ${kido_common_srcs}
  ${kido_math_srcs}
  ${kido_integration_srcs}
  ${kido_lcpsolver_srcs}
  ${kido_dynamics_srcs}
  ${kido_optimizer_srcs}
  ${kido_collision_srcs}
  ${kido_constraint_srcs}
  ${kido_renderer_srcs}
  ${kido_simulation_srcs}
)

# Library: kido
kido_add_library(kido ${kido_hdrs} ${kido_srcs})
target_link_libraries(kido ${KIDO_DEPENDENCIES})

if(MSVC)
  set_target_properties(
    ${target} PROPERTIES
    STATIC_LIBRARY_FLAGS_RELEASE "/LTCG"
  )
endif()

install(FILES kido.hpp DESTINATION include/kido/ COMPONENT headers)
install(TARGETS kido EXPORT KIDOTargets DESTINATION lib)
