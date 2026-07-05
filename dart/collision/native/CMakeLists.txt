# Copyright (c) 2011, The DART development contributors

set(
  collision_native_headers
  ${CMAKE_CURRENT_SOURCE_DIR}/export.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/fwd.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/aabb.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/contact_point.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/types.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/contact_manifold.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/detail/span.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/shapes/shape.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sdf/signed_distance_field.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/gjk.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/gjk_inl.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/mpr.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box/sat.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box/face_clip.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box/contact_reduction.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/sphere_sphere.hpp
)

set(
  collision_native_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/aabb.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/types.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/contact_manifold.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/shapes/shape.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/gjk.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/mpr.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box/sat.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box/face_clip.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/box_box/contact_reduction.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/narrow_phase/sphere_sphere.cpp
)

dart_add_core_headers(${collision_native_headers})
dart_add_core_sources(${collision_native_sources})
