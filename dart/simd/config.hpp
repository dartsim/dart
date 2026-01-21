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

#include <cstddef>
#include <cstdint>

#if defined(DART_SIMD_FORCE_SCALAR)
  #undef DART_SIMD_AVX512
  #undef DART_SIMD_AVX2
  #undef DART_SIMD_AVX
  #undef DART_SIMD_SSE42
  #undef DART_SIMD_NEON
  #undef DART_SIMD_FMA
  #define DART_SIMD_SCALAR 1
#elif defined(__AVX512F__) && defined(__AVX512DQ__)
  #define DART_SIMD_AVX512 1
  #define DART_SIMD_AVX2 1
  #define DART_SIMD_AVX 1
  #define DART_SIMD_SSE42 1
#elif defined(__AVX2__) && defined(__FMA__)
  #define DART_SIMD_AVX2 1
  #define DART_SIMD_AVX 1
  #define DART_SIMD_SSE42 1
#elif defined(__AVX__)
  #define DART_SIMD_AVX 1
  #define DART_SIMD_SSE42 1
#elif defined(__SSE4_2__)
  #define DART_SIMD_SSE42 1
#endif

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
  #define DART_SIMD_NEON 1
#endif

#if defined(__FMA__) || defined(DART_SIMD_AVX2)
  #define DART_SIMD_FMA 1
#endif

#if !defined(DART_SIMD_SSE42) && !defined(DART_SIMD_NEON)
  #define DART_SIMD_SCALAR 1
#endif

#if defined(DART_SIMD_SSE42)
  #include <immintrin.h>
  #if defined(_MSC_VER)
    #include <intrin.h>
  #endif
#endif

#if defined(DART_SIMD_NEON)
  #include <arm_neon.h>
#endif

#if defined(__GNUC__) || defined(__clang__)
  #define DART_SIMD_INLINE __attribute__((always_inline)) inline
  #define DART_SIMD_ALIGNED(x) __attribute__((aligned(x)))
#elif defined(_MSC_VER)
  #define DART_SIMD_INLINE __forceinline
  #define DART_SIMD_ALIGNED(x) __declspec(align(x))
#else
  #define DART_SIMD_INLINE inline
  #define DART_SIMD_ALIGNED(x)
#endif

namespace dart::simd {

#if defined(DART_SIMD_AVX512)
inline constexpr const char* backend_name = "AVX-512";
inline constexpr std::size_t max_vector_bytes = 64;
#elif defined(DART_SIMD_AVX2)
inline constexpr const char* backend_name = "AVX2+FMA";
inline constexpr std::size_t max_vector_bytes = 32;
#elif defined(DART_SIMD_AVX)
inline constexpr const char* backend_name = "AVX";
inline constexpr std::size_t max_vector_bytes = 32;
#elif defined(DART_SIMD_SSE42)
inline constexpr const char* backend_name = "SSE4.2";
inline constexpr std::size_t max_vector_bytes = 16;
#elif defined(DART_SIMD_NEON)
inline constexpr const char* backend_name = "NEON";
inline constexpr std::size_t max_vector_bytes = 16;
#else
inline constexpr const char* backend_name = "Scalar";
inline constexpr std::size_t max_vector_bytes = 8;
#endif

namespace detail {

template <typename T>
struct NativeWidth
{
  static constexpr std::size_t value = 1;
};

#if defined(DART_SIMD_AVX512)
template <>
struct NativeWidth<float>
{
  static constexpr std::size_t value = 16;
};
template <>
struct NativeWidth<double>
{
  static constexpr std::size_t value = 8;
};
template <>
struct NativeWidth<std::int32_t>
{
  static constexpr std::size_t value = 16;
};
template <>
struct NativeWidth<std::uint32_t>
{
  static constexpr std::size_t value = 16;
};
#elif defined(DART_SIMD_AVX2) || defined(DART_SIMD_AVX)
template <>
struct NativeWidth<float>
{
  static constexpr std::size_t value = 8;
};
template <>
struct NativeWidth<double>
{
  static constexpr std::size_t value = 4;
};
template <>
struct NativeWidth<std::int32_t>
{
  static constexpr std::size_t value = 8;
};
template <>
struct NativeWidth<std::uint32_t>
{
  static constexpr std::size_t value = 8;
};
#elif defined(DART_SIMD_SSE42)
template <>
struct NativeWidth<float>
{
  static constexpr std::size_t value = 4;
};
template <>
struct NativeWidth<double>
{
  static constexpr std::size_t value = 2;
};
template <>
struct NativeWidth<std::int32_t>
{
  static constexpr std::size_t value = 4;
};
template <>
struct NativeWidth<std::uint32_t>
{
  static constexpr std::size_t value = 4;
};
#elif defined(DART_SIMD_NEON)
template <>
struct NativeWidth<float>
{
  static constexpr std::size_t value = 4;
};
template <>
struct NativeWidth<double>
{
  static constexpr std::size_t value = 2;
};
template <>
struct NativeWidth<std::int32_t>
{
  static constexpr std::size_t value = 4;
};
template <>
struct NativeWidth<std::uint32_t>
{
  static constexpr std::size_t value = 4;
};
#endif

} // namespace detail

template <typename T>
inline constexpr std::size_t preferred_width_v = detail::NativeWidth<T>::value;

#if defined(DART_SIMD_SSE42)

DART_SIMD_INLINE void set_flush_denormals(bool enable) noexcept
{
  if (enable) {
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
  } else {
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_OFF);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_OFF);
  }
}

[[nodiscard]] DART_SIMD_INLINE bool flush_denormals() noexcept
{
  return _MM_GET_FLUSH_ZERO_MODE() == _MM_FLUSH_ZERO_ON;
}

#else

DART_SIMD_INLINE void set_flush_denormals(bool) noexcept {}

[[nodiscard]] DART_SIMD_INLINE bool flush_denormals() noexcept
{
  return false;
}

#endif

class ScopedFlushDenormals
{
public:
  explicit ScopedFlushDenormals(bool enable) noexcept
    : previous_state_(flush_denormals())
  {
    set_flush_denormals(enable);
  }

  ~ScopedFlushDenormals() noexcept
  {
    set_flush_denormals(previous_state_);
  }

  ScopedFlushDenormals(const ScopedFlushDenormals&) = delete;
  ScopedFlushDenormals& operator=(const ScopedFlushDenormals&) = delete;

private:
  bool previous_state_;
};

} // namespace dart::simd
