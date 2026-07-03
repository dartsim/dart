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

#include <type_traits>
#include <utility>

#include <cstddef>
#include <cstdint>

namespace dart::simd {

template <typename T, std::size_t Width>
struct Vec;

template <typename T, std::size_t Width>
struct VecMask;

// C++17 trait equivalents of the DART 7 dart/simd concepts (ScalarType,
// FloatType, IntType, SignedType, ValidWidth, IsVec, IsVecMask); DART 6
// builds with C++17, which has no `concept`.

template <typename T>
inline constexpr bool is_scalar_type_v
    = std::is_same_v<
          T,
          float> || std::is_same_v<T, double> || std::is_same_v<T, std::int32_t> || std::is_same_v<T, std::uint32_t>;

template <typename T>
inline constexpr bool is_float_type_v
    = std::is_same_v<T, float> || std::is_same_v<T, double>;

template <typename T>
inline constexpr bool is_int_type_v
    = std::is_same_v<T, std::int32_t> || std::is_same_v<T, std::uint32_t>;

template <typename T>
inline constexpr bool is_signed_type_v
    = std::is_same_v<
          T,
          float> || std::is_same_v<T, double> || std::is_same_v<T, std::int32_t>;

template <std::size_t W>
inline constexpr bool is_valid_width_v
    = (W == 1) || (W == 2) || (W == 4) || (W == 8) || (W == 16);

namespace detail {

template <typename V, typename = void>
struct IsVecTrait : std::false_type
{
};

template <typename V>
struct IsVecTrait<V, std::void_t<typename V::scalar_type, decltype(V::width)>>
  : std::bool_constant<
        std::is_convertible_v<
            decltype(V::width),
            std::
                size_t> && is_scalar_type_v<typename V::scalar_type> && is_valid_width_v<static_cast<std::size_t>(V::width)>>
{
};

template <typename M, typename = void>
struct IsVecMaskTrait : std::false_type
{
};

template <typename M>
struct IsVecMaskTrait<
    M,
    std::void_t<
        typename M::scalar_type,
        decltype(M::width),
        decltype(std::declval<const M&>().all()),
        decltype(std::declval<const M&>().any())>>
  : std::bool_constant<
        std::is_convertible_v<decltype(M::width), std::size_t>
        && std::is_convertible_v<decltype(std::declval<const M&>().all()), bool>
        && std::is_convertible_v<
            decltype(std::declval<const M&>().any()),
            bool>>
{
};

} // namespace detail

template <typename V>
inline constexpr bool is_vec_v = detail::IsVecTrait<V>::value;

template <typename M>
inline constexpr bool is_vec_mask_v = detail::IsVecMaskTrait<M>::value;

using Vec2f = Vec<float, 2>;
using Vec4f = Vec<float, 4>;
using Vec8f = Vec<float, 8>;
using Vec16f = Vec<float, 16>;

using Vec2d = Vec<double, 2>;
using Vec4d = Vec<double, 4>;
using Vec8d = Vec<double, 8>;

using Vec4i = Vec<std::int32_t, 4>;
using Vec8i = Vec<std::int32_t, 8>;
using Vec16i = Vec<std::int32_t, 16>;

using Vec4u = Vec<std::uint32_t, 4>;
using Vec8u = Vec<std::uint32_t, 8>;
using Vec16u = Vec<std::uint32_t, 16>;

using VecNf = Vec<float, preferred_width_v<float>>;
using VecNd = Vec<double, preferred_width_v<double>>;
using VecNi = Vec<std::int32_t, preferred_width_v<std::int32_t>>;
using VecNu = Vec<std::uint32_t, preferred_width_v<std::uint32_t>>;

template <typename VecT>
using vec_mask_t = VecMask<typename VecT::scalar_type, VecT::width>;

using VecMask4f = VecMask<float, 4>;
using VecMask8f = VecMask<float, 8>;
using VecMask2d = VecMask<double, 2>;
using VecMask4d = VecMask<double, 4>;
using VecMask4i = VecMask<std::int32_t, 4>;
using VecMask8i = VecMask<std::int32_t, 8>;

} // namespace dart::simd
