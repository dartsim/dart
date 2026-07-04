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

#include <dart/simd/simd.hpp>

#include <gtest/gtest.h>

#include <iostream>

#include <cstring>

using namespace dart::simd;

// Compile-time checks for the C++17 trait ports of the DART 7 concepts: a
// width member that is not a constant expression must yield false (not a
// hard error), and non-const all()/any() must still be detected, matching
// the DART 7 IsVec/IsVecMask semantics.
namespace {

struct NonConstantWidth
{
  using scalar_type = float;
  std::size_t width;
};

struct RuntimeStaticWidth
{
  using scalar_type = float;
  static std::size_t width;
};

std::size_t RuntimeStaticWidth::width = 4;

struct InvalidWidth
{
  using scalar_type = float;
  static constexpr std::size_t width = 3;
};

struct NonConstMask
{
  using scalar_type = float;
  static constexpr std::size_t width = 4;
  bool all()
  {
    return true;
  }
  bool any()
  {
    return true;
  }
};

#if defined(DART_SIMD_SSE42)
struct MxcsrGuard
{
  unsigned int value;

  ~MxcsrGuard()
  {
    _mm_setcsr(value);
  }
};
#endif

} // namespace

static_assert(is_vec_v<Vec4f>, "Vec4f must satisfy is_vec_v");
static_assert(!is_vec_v<int>, "int must not satisfy is_vec_v");
static_assert(
    !is_vec_v<NonConstantWidth>,
    "a non-constant width member must yield false, not a hard error");
static_assert(
    !is_vec_v<RuntimeStaticWidth>,
    "a runtime static width member must yield false, not a hard error");
static_assert(
    !is_vec_v<InvalidWidth>, "invalid vector widths must be rejected");
static_assert(is_vec_mask_v<VecMask4f>, "VecMask4f must satisfy is_vec_mask_v");
static_assert(!is_vec_mask_v<int>, "int must not satisfy is_vec_mask_v");
static_assert(
    is_vec_mask_v<NonConstMask>,
    "non-const all()/any() must be detected, matching DART 7's requires(M m)");

TEST(SimdConfig, BackendName)
{
  EXPECT_NE(backend_name, nullptr);
  EXPECT_GT(std::strlen(backend_name), 0u);

  std::cout << "SIMD backend: " << backend_name << std::endl;
  std::cout << "Max vector bytes: " << max_vector_bytes << std::endl;
}

TEST(SimdConfig, PreferredWidths)
{
  EXPECT_GE(preferred_width_v<float>, 1u);
  EXPECT_GE(preferred_width_v<double>, 1u);
  EXPECT_GE(preferred_width_v<std::int32_t>, 1u);

  std::cout << "Preferred width (float): "
            << preferred_width_v<float> << std::endl;
  std::cout << "Preferred width (double): "
            << preferred_width_v<double> << std::endl;
  std::cout << "Preferred width (int32): "
            << preferred_width_v<std::int32_t> << std::endl;
}

// Check for highest instruction set first (AVX2 > AVX > SSE42 > NEON > Scalar)
// because lower instruction sets may be defined alongside higher ones
#if defined(DART_SIMD_AVX2)

TEST(SimdConfig, AVX2Detected)
{
  EXPECT_STREQ(backend_name, "AVX2+FMA");
  EXPECT_EQ(max_vector_bytes, 32u);
  EXPECT_EQ(preferred_width_v<float>, 8u);
  EXPECT_EQ(preferred_width_v<double>, 4u);
  EXPECT_EQ(preferred_width_v<std::int32_t>, 8u);
}

#elif defined(DART_SIMD_AVX)

TEST(SimdConfig, AVXDetected)
{
  EXPECT_STREQ(backend_name, "AVX");
  EXPECT_EQ(max_vector_bytes, 32u);
  EXPECT_EQ(preferred_width_v<float>, 8u);
  EXPECT_EQ(preferred_width_v<double>, 4u);
}

#elif defined(DART_SIMD_SSE42)

TEST(SimdConfig, SSE42Detected)
{
  EXPECT_STREQ(backend_name, "SSE4.2");
  EXPECT_EQ(max_vector_bytes, 16u);
  EXPECT_EQ(preferred_width_v<float>, 4u);
  EXPECT_EQ(preferred_width_v<double>, 2u);
  EXPECT_EQ(preferred_width_v<std::int32_t>, 4u);
}

TEST(SimdConfig, SSE42VecUsesIntrinsics)
{
  EXPECT_EQ(sizeof(Vec<float, 4>), sizeof(__m128));
  EXPECT_EQ(sizeof(Vec<double, 2>), sizeof(__m128d));
  EXPECT_EQ(sizeof(Vec<std::int32_t, 4>), sizeof(__m128i));
}

#elif defined(DART_SIMD_NEON)

TEST(SimdConfig, NEONDetected)
{
  EXPECT_STREQ(backend_name, "NEON");
  EXPECT_EQ(max_vector_bytes, 16u);
  EXPECT_EQ(preferred_width_v<float>, 4u);
  EXPECT_EQ(preferred_width_v<double>, 2u);
}

#else

TEST(SimdConfig, ScalarFallback)
{
  EXPECT_STREQ(backend_name, "Scalar");
}

#endif

TEST(SimdConfig, FlushDenormals)
{
#if defined(DART_SIMD_SSE42)
  MxcsrGuard guard{_mm_getcsr()};
#endif
  bool original = flush_denormals();

  set_flush_denormals(true);
#if defined(DART_SIMD_SSE42)
  EXPECT_TRUE(flush_denormals());
#endif

  set_flush_denormals(false);
#if defined(DART_SIMD_SSE42)
  EXPECT_FALSE(flush_denormals());
#endif

  set_flush_denormals(original);
}

TEST(SimdConfig, ScopedFlushDenormals)
{
#if defined(DART_SIMD_SSE42)
  MxcsrGuard guard{_mm_getcsr()};
#endif
  bool original = flush_denormals();
  set_flush_denormals(false);

  {
    ScopedFlushDenormals guard(true);
#if defined(DART_SIMD_SSE42)
    EXPECT_TRUE(flush_denormals());
#endif
  }

#if defined(DART_SIMD_SSE42)
  EXPECT_FALSE(flush_denormals());
#endif

  set_flush_denormals(original);
}

#if defined(DART_SIMD_SSE42)
TEST(SimdConfig, ScopedFlushDenormalsRestoresMxcsrModes)
{
  constexpr unsigned int denormalsZeroMask = 0x0040u;
  constexpr unsigned int flushZeroMask = 0x8000u;
  constexpr unsigned int denormalModeMask = denormalsZeroMask | flushZeroMask;
  MxcsrGuard guard{_mm_getcsr()};

  const unsigned int base = guard.value & ~denormalModeMask;
  const unsigned int states[]
      = {base | flushZeroMask, base | denormalsZeroMask};
  for (const unsigned int expected : states) {
    _mm_setcsr(expected);
    {
      ScopedFlushDenormals scoped(true);
      EXPECT_EQ(_mm_getcsr() & denormalModeMask, denormalModeMask);
    }
    EXPECT_EQ(_mm_getcsr() & denormalModeMask, expected & denormalModeMask);
  }
}
#endif
