# Copyright (c) 2011, The DART development contributors

include(GNUInstallDirs)

set(
  collision_native_headers
  ${CMAKE_CURRENT_SOURCE_DIR}/Export.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Fwd.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Aabb.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ContactPoint.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Types.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ContactManifold.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/NativeCollisionDetector.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/NativeCollisionGroup.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/NativeCollisionObject.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/detail/Span.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/detail/NativeShapeConversion.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/shapes/Shape.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sdf/SignedDistanceField.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/broad_phase/BroadPhase.hpp
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
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/ConvexConvex.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/CylinderCollision.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Distance.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/MeshMesh.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/PlaneSphere.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/SphereBox.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/SphereSphere.hpp
)

foreach(native_header ${collision_native_headers})
  file(
    RELATIVE_PATH
    native_header_rel
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${native_header}"
  )
  get_filename_component(native_header_dir "${native_header_rel}" DIRECTORY)

  if(native_header_dir STREQUAL "")
    set(
      native_header_destination
      "${CMAKE_INSTALL_INCLUDEDIR}/dart/collision/native"
    )
  else()
    set(
      native_header_destination
      "${CMAKE_INSTALL_INCLUDEDIR}/dart/collision/native/${native_header_dir}"
    )
  endif()

  install(
    FILES "${native_header}"
    DESTINATION "${native_header_destination}"
    COMPONENT headers
  )
endforeach()

set(
  collision_native_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/Aabb.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Types.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ContactManifold.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/NativeCollisionDetector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/NativeCollisionGroup.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/NativeCollisionObject.cpp
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
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/ConvexConvex.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/CylinderCollision.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/Distance.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/MeshMesh.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/PlaneSphere.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/SphereBox.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/SphereSphere.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/broad_phase/BruteForce.cpp
)

dart_add_core_headers(${collision_native_headers})
dart_add_core_sources(${collision_native_sources})
