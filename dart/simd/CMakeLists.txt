# Copyright (c) 2011, The DART development contributors

set(
  simd_headers
  Config.hpp
  Fwd.hpp
  Math.hpp
  Memory.hpp
  Simd.hpp
  detail/Backend.hpp
  # Scalar backend (always available)
  detail/scalar/Vec.hpp
  detail/scalar/VecMask.hpp
  detail/scalar/Operations.hpp
  # SSE4.2 backend
  detail/sse42/Vec.hpp
  detail/sse42/VecMask.hpp
  detail/sse42/Operations.hpp
  # AVX backend
  detail/avx/Vec.hpp
  detail/avx/VecMask.hpp
  detail/avx/Operations.hpp
  # AVX2 backend
  detail/avx2/Vec.hpp
  detail/avx2/VecMask.hpp
  detail/avx2/Operations.hpp
  # AVX-512 backend
  detail/avx512/Vec.hpp
  detail/avx512/VecMask.hpp
  detail/avx512/Operations.hpp
  # ARM NEON backend
  detail/neon/Vec.hpp
  detail/neon/VecMask.hpp
  detail/neon/Operations.hpp
  # ARM SVE backend
  detail/sve/Vec.hpp
  detail/sve/VecMask.hpp
  detail/sve/Operations.hpp
  # Math functions
  detail/math/Constants.hpp
  detail/math/ExpImpl.hpp
  detail/math/LogImpl.hpp
  detail/math/Polynomial.hpp
  detail/math/SincosImpl.hpp
  # Geometry types
  geometry/Vector3.hpp
  geometry/Vector4.hpp
  geometry/Matrix3x3.hpp
  geometry/Matrix4x4.hpp
  geometry/Quaternion.hpp
  geometry/Isometry3.hpp
  geometry/Batch.hpp
  # Dynamic types
  dynamic/Vector.hpp
  dynamic/Matrix.hpp
  # Eigen interop
  eigen/Interop.hpp
  eigen/Iterator.hpp
)

# Header-only library (interface target)
add_library(dart-simd INTERFACE)
target_include_directories(
  dart-simd
  INTERFACE
    $<BUILD_INTERFACE:${DART_SOURCE_DIR}>
    $<BUILD_INTERFACE:${DART_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)
# DART 7 declares cxx_std_23 here; the DART 6 port is adapted to C++17 to
# preserve the LTS toolchain contract for downstream consumers.
target_compile_features(dart-simd INTERFACE cxx_std_17)
target_link_libraries(dart-simd INTERFACE Eigen3::Eigen)

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
