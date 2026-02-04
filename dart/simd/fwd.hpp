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

#include <concepts>
#include <type_traits>

#include <cstddef>
#include <cstdint>

namespace dart::simd {

template <typename T, std::size_t Width>
struct Vec;

template <typename T, std::size_t Width>
struct VecMask;

template <typename T>
concept ScalarType
    = std::same_as<T, float> || std::same_as<T, double>
      || std::same_as<T, std::int32_t> || std::same_as<T, std::uint32_t>;

template <typename T>
concept FloatType = std::same_as<T, float> || std::same_as<T, double>;

template <typename T>
concept IntType
    = std::same_as<T, std::int32_t> || std::same_as<T, std::uint32_t>;

template <typename T>
concept SignedType = std::same_as<T, float> || std::same_as<T, double>
                     || std::same_as<T, std::int32_t>;

template <std::size_t W>
concept ValidWidth = (W == 1) || (W == 2) || (W == 4) || (W == 8) || (W == 16);

template <typename V>
concept IsVec = requires {
  typename V::scalar_type;
  { V::width } -> std::convertible_to<std::size_t>;
} && ScalarType<typename V::scalar_type> && ValidWidth<V::width>;

template <typename M>
concept IsVecMask = requires(M m) {
  typename M::scalar_type;
  { M::width } -> std::convertible_to<std::size_t>;
  { m.all() } -> std::convertible_to<bool>;
  { m.any() } -> std::convertible_to<bool>;
};

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
