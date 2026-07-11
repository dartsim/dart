# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs})
dart_add_core_sources(${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "collision_dart headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/dart.hpp"
  "dart/collision/dart/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/dart.hpp
  DESTINATION include/dart/collision/dart
  COMPONENT headers
)

# Engine internals absorbed from the former dart/collision/native/ (the
# native engine merged into this detector; see the maintainer-directed
# consolidation). These stay under the dart::collision::native namespace to
# minimize diff noise; only the detector-facing classes changed names.
set(
  collision_dart_engine_headers
  ${CMAKE_CURRENT_SOURCE_DIR}/detail/Span.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/detail/NativeShapeConversion.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/shapes/Shape.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sdf/SignedDistanceField.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/broad_phase/BroadPhase.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/broad_phase/AabbTree.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/broad_phase/BruteForce.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Gjk.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Gjk-impl.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Mpr.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/NarrowPhase.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/BoxBox.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box/Sat.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box/FaceClip.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box/ContactReduction.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/CapsuleBox.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/CapsuleCapsule.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/CapsuleSphere.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Ccd.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/ConvexConvex.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/CylinderCollision.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Distance.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/MeshMesh.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/PlaneSphere.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Raycast.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/PrimitiveCcd.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/SphereBox.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/SphereSphere.hpp
)

foreach(engine_header ${collision_dart_engine_headers})
  file(
    RELATIVE_PATH
    engine_header_rel
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${engine_header}"
  )
  get_filename_component(engine_header_dir "${engine_header_rel}" DIRECTORY)

  install(
    FILES "${engine_header}"
    DESTINATION
      "${CMAKE_INSTALL_INCLUDEDIR}/dart/collision/dart/${engine_header_dir}"
    COMPONENT headers
  )
endforeach()

set(
  collision_dart_engine_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/detail/NativeShapeConversion.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/shapes/Shape.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Gjk.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Mpr.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/NarrowPhase.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/BoxBox.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box/Sat.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box/FaceClip.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box/ContactReduction.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/CapsuleBox.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/CapsuleCapsule.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/CapsuleSphere.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Ccd.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/ConvexConvex.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/CylinderCollision.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Distance.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/MeshMesh.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/PlaneSphere.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Raycast.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/PrimitiveCcd.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/SphereBox.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/SphereSphere.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/broad_phase/AabbTree.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/broad_phase/BruteForce.cpp
)

dart_add_core_headers(${collision_dart_engine_headers})
dart_add_core_sources(${collision_dart_engine_sources})
