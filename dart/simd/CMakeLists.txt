# Copyright (c) 2011, The DART development contributors

set(simd_headers
  config.hpp
  fwd.hpp
  memory.hpp
  simd.hpp
  detail/scalar/vec.hpp
  detail/scalar/vec_mask.hpp
  detail/scalar/operations.hpp
  geometry/vector3.hpp
  geometry/vector4.hpp
  geometry/matrix3x3.hpp
  geometry/matrix4x4.hpp
  geometry/quaternion.hpp
  geometry/isometry3.hpp
  dynamic/vector.hpp
  dynamic/matrix.hpp
  eigen/interop.hpp
  eigen/iterator.hpp
)

# Header-only library (interface target)
add_library(dart-simd INTERFACE)
target_include_directories(
  dart-simd INTERFACE
    $<BUILD_INTERFACE:${DART_SOURCE_DIR}>
    $<BUILD_INTERFACE:${DART_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(dart-simd INTERFACE cxx_std_20)

# Install headers
foreach(header ${simd_headers})
  get_filename_component(header_dir ${header} DIRECTORY)
  install(
    FILES ${header}
    DESTINATION include/dart/simd/${header_dir}
    COMPONENT headers
  )
endforeach()

dart_format_add(${simd_headers})
