/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <dart/simd/config.hpp>
#include <dart/simd/detail/scalar/operations.hpp>
#include <dart/simd/detail/scalar/vec.hpp>
#include <dart/simd/detail/scalar/vec_mask.hpp>
#include <dart/simd/fwd.hpp>
#include <dart/simd/memory.hpp>

#if defined(DART_SIMD_SSE42)
  #include <dart/simd/detail/sse42/operations.hpp>
  #include <dart/simd/detail/sse42/vec.hpp>
  #include <dart/simd/detail/sse42/vec_mask.hpp>
#endif

#if defined(DART_SIMD_AVX) && !defined(DART_SIMD_AVX2)
  #include <dart/simd/detail/avx/operations.hpp>
  #include <dart/simd/detail/avx/vec.hpp>
  #include <dart/simd/detail/avx/vec_mask.hpp>
#endif

#if defined(DART_SIMD_AVX2)
  #include <dart/simd/detail/avx2/operations.hpp>
  #include <dart/simd/detail/avx2/vec.hpp>
  #include <dart/simd/detail/avx2/vec_mask.hpp>
#endif

#if defined(DART_SIMD_SVE)
  #include <dart/simd/detail/sve/operations.hpp>
  #include <dart/simd/detail/sve/vec.hpp>
  #include <dart/simd/detail/sve/vec_mask.hpp>
#elif defined(DART_SIMD_NEON)
  #include <dart/simd/detail/neon/operations.hpp>
  #include <dart/simd/detail/neon/vec.hpp>
  #include <dart/simd/detail/neon/vec_mask.hpp>
#endif

#if defined(DART_SIMD_AVX512)
  #include <dart/simd/detail/avx512/operations.hpp>
  #include <dart/simd/detail/avx512/vec.hpp>
  #include <dart/simd/detail/avx512/vec_mask.hpp>
#endif

#include <dart/simd/dynamic/matrix.hpp>
#include <dart/simd/dynamic/vector.hpp>
#include <dart/simd/eigen/interop.hpp>
#include <dart/simd/geometry/isometry3.hpp>
#include <dart/simd/geometry/matrix3x3.hpp>
#include <dart/simd/geometry/matrix4x4.hpp>
#include <dart/simd/geometry/quaternion.hpp>
#include <dart/simd/geometry/vector3.hpp>
#include <dart/simd/geometry/vector4.hpp>
