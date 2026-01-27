# Copyright (c) 2011, The DART development contributors

set(simd_headers
  config.hpp
  fwd.hpp
  memory.hpp
  simd.hpp
  # Scalar backend (always available)
  detail/scalar/vec.hpp
  detail/scalar/vec_mask.hpp
  detail/scalar/operations.hpp
  # SSE4.2 backend
  detail/sse42/vec.hpp
  detail/sse42/vec_mask.hpp
  detail/sse42/operations.hpp
  # AVX backend
  detail/avx/vec.hpp
  detail/avx/vec_mask.hpp
  detail/avx/operations.hpp
  # AVX2 backend
  detail/avx2/vec.hpp
  detail/avx2/vec_mask.hpp
  detail/avx2/operations.hpp
  # AVX-512 backend
  detail/avx512/vec.hpp
  detail/avx512/vec_mask.hpp
  detail/avx512/operations.hpp
  # ARM NEON backend
  detail/neon/vec.hpp
  detail/neon/vec_mask.hpp
  detail/neon/operations.hpp
  # ARM SVE backend
  detail/sve/vec.hpp
  detail/sve/vec_mask.hpp
  detail/sve/operations.hpp
  # Geometry types
  geometry/vector3.hpp
  geometry/vector4.hpp
  geometry/matrix3x3.hpp
  geometry/matrix4x4.hpp
  geometry/quaternion.hpp
  geometry/isometry3.hpp
  geometry/batch.hpp
  # Dynamic types
  dynamic/vector.hpp
  dynamic/matrix.hpp
  # Eigen interop
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
