# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
file(GLOB dart_hdrs "dart/*.hpp")
set(
  fcl_hdrs
  "${CMAKE_CURRENT_SOURCE_DIR}/fcl/BackwardCompatibility.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/fcl/FCLCollisionDetector.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/fcl/FCLCollisionGroup.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/fcl/FCLCollisionObject.hpp"
)
set(
  ode_hdrs
  "${CMAKE_CURRENT_SOURCE_DIR}/ode/OdeCollisionDetector.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/ode/OdeCollisionGroup.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/ode/OdeCollisionObject.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/ode/OdeStub.hpp"
)
set(
  bullet_hdrs
  "${CMAKE_CURRENT_SOURCE_DIR}/bullet/BulletCollisionDetector.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/bullet/BulletCollisionGroup.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/bullet/BulletCollisionObject.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/bullet/BulletCollisionShape.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/bullet/BulletInclude.hpp"
)
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "collision headers" ${hdrs})
list(REMOVE_ITEM header_names SmartPointer.hpp)
set(
  header_names
  ${header_names}
  dart/All.hpp
  fcl/All.hpp
)
dart_generate_component_headers(
  COMPONENT_NAME collision
  TARGET_DIR "dart/collision/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs} ${detail_hdrs} ${dart_hdrs} ${fcl_hdrs} ${ode_hdrs} ${bullet_hdrs}
)

# Install headers and generated component headers
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/collision.hpp
  DESTINATION include/dart/collision
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/collision/detail
  COMPONENT headers
)

# Install generated PascalCase compatibility headers, preserving directory structure
dart_install_compat_headers(COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS} DESTINATION_PREFIX include/dart/collision)

# Add subdirectories
add_subdirectory(dart)
add_subdirectory(fcl)
add_subdirectory(ode)
add_subdirectory(bullet)
