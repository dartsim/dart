/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

// Batch (data-parallel) operations for the typed Lie group API.
//
// These are *consolidated* with the single-element Lie group types rather than a
// parallel SoA class hierarchy: each function views a packed, contiguous buffer
// element-wise through Eigen::Map<SO3<S>> / Eigen::Map<SE3<S>> and applies the
// existing scalar operation. Buffers are AoS of `Params` (each element occupies
// `T::ParamSize` scalars; tangents occupy `T::DoF` scalars), matching the
// layout the Map specializations expect.
//
// The implementations below are the always-correct scalar reference path. SIMD
// acceleration (see dart/simd) is expected to be slotted in *behind this same
// interface* (kept off the public type surface, per
// docs/design/scalable_compute_decisions.md and docs/onboarding/api-boundaries.md).
// Escalating to a dedicated SoA storage type should only happen if benchmarks on
// the experimental engine prove the interleaved Map layout is the bottleneck.
// See docs/design/lie_group_batch.md.

#include <dart/math/lie_group/functions.hpp>
#include <dart/math/lie_group/se3.hpp>
#include <dart/math/lie_group/so3.hpp>

#include <cstddef>

namespace dart::math {

/// Batch group composition: out[i] = lhs[i] * rhs[i] for i in [0, count).
///
/// @tparam T A Lie group type (e.g., SO3<double>, SE3<float>).
/// @param[in] lhs Packed left operands (count * T::ParamSize scalars).
/// @param[in] rhs Packed right operands (count * T::ParamSize scalars).
/// @param[out] out Packed results (count * T::ParamSize scalars). May alias lhs.
/// @param[in] count Number of elements.
template <typename T>
void composeBatch(
    const typename T::Scalar* lhs,
    const typename T::Scalar* rhs,
    typename T::Scalar* out,
    std::size_t count)
{
  constexpr int P = T::ParamSize;
  for (std::size_t i = 0; i < count; ++i) {
    const ::Eigen::Map<const T> a(lhs + i * P);
    const ::Eigen::Map<const T> b(rhs + i * P);
    ::Eigen::Map<T> o(out + i * P);
    o = a * b;
  }
}

/// Batch exponential map: out[i] = Exp(tangents[i]) for i in [0, count).
///
/// @tparam T A Lie group type (e.g., SO3<double>, SE3<float>).
/// @param[in] tangents Packed tangent vectors (count * T::DoF scalars).
/// @param[out] out Packed group elements (count * T::ParamSize scalars).
/// @param[in] count Number of elements.
template <typename T>
void expBatch(
    const typename T::Scalar* tangents,
    typename T::Scalar* out,
    std::size_t count)
{
  using S = typename T::Scalar;
  constexpr int P = T::ParamSize;
  constexpr int D = T::DoF;
  for (std::size_t i = 0; i < count; ++i) {
    const typename T::Tangent tangent(
        ::Eigen::Map<const ::Eigen::Matrix<S, D, 1>>(tangents + i * D));
    ::Eigen::Map<T> o(out + i * P);
    o = ::dart::math::Exp(tangent);
  }
}

/// Batch logarithm map: out[i] = Log(groups[i]) for i in [0, count).
///
/// @tparam T A Lie group type (e.g., SO3<double>, SE3<float>).
/// @param[in] groups Packed group elements (count * T::ParamSize scalars).
/// @param[out] out Packed tangent vectors (count * T::DoF scalars).
/// @param[in] count Number of elements.
template <typename T>
void logBatch(
    const typename T::Scalar* groups,
    typename T::Scalar* out,
    std::size_t count)
{
  using S = typename T::Scalar;
  constexpr int P = T::ParamSize;
  constexpr int D = T::DoF;
  for (std::size_t i = 0; i < count; ++i) {
    const ::Eigen::Map<const T> g(groups + i * P);
    ::Eigen::Map<::Eigen::Matrix<S, D, 1>>(out + i * D)
        = ::dart::math::Log(g).params();
  }
}

} // namespace dart::math
