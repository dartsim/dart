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

// WP-PG.40 prototype (design packet, docs/dev_tasks/dart6_performance_generalization).
// Not part of the installed dart/math API; see 08-simd-contracts.md D1 for the
// decision this demonstrates. Placement rationale: this sketch belongs to the
// design packet, not the dart/math seam -- WP-PG.41 owns picking the real
// header location and integrating with contact-Jacobian call sites once the
// D1/D2 decisions are ratified.
//
// Demonstrates the FP-determinism contract proposed in 08-simd-contracts.md
// (D1) using a kernel that already exists and is already unconsumed: the
// 4-wide SoA batch cross product at dart::simd::cross(Vector3SoA<T,N>)
// (dart/simd/geometry/batch.hpp:536-544). That kernel is implemented with
// fmsub(a.y, b.z, a.z * b.y) == fma(a.y, b.z, -(a.z * b.y)) -- i.e. it
// already commits to a fused-multiply-subtract policy.
//
// This header does not add a new kernel; it packages two scalar references
// for that existing kernel plus a bit-equality comparator, so the harness in
// wp_pg_40_cross3_batch_test.cpp can be pointed at the ci_simd.yml
// DART_SIMD_FORCE_SCALAR / sse42 / avx / avx2 matrix later (WP-PG.41) with no
// further rewriting.

#include <dart/simd/config.hpp>
#include <dart/simd/geometry/batch.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>

namespace wp_pg_40 {

using dart::simd::Vec;
using dart::simd::Vector3SoA;

/// Reference A -- "naive scalar": infix `a*b - c*d`, no operation fusion.
/// This is the same formula, term for term, as the *existing* AoS
/// `dart::simd::Vector3<T>::cross()` (dart/simd/geometry/vector3.hpp:322-331):
/// `cx = a.y()*b.z() - a.z()*b.y()`. It is included here because that AoS
/// formula is the thing a reviewer would reach for as "the scalar reference"
/// -- and the D1 finding is that it is *not* bit-identical to the SoA batch
/// kernel already merged in the same module.
template <typename T, std::size_t N>
[[nodiscard]] Vector3SoA<T, N> scalarCrossNaive(
    const Vector3SoA<T, N>& a, const Vector3SoA<T, N>& b) noexcept
{
  std::array<T, N> ax{}, ay{}, az{}, bx{}, by{}, bz{}, rx{}, ry{}, rz{};
  a.x.store(ax.data());
  a.y.store(ay.data());
  a.z.store(az.data());
  b.x.store(bx.data());
  b.y.store(by.data());
  b.z.store(bz.data());

  for (std::size_t i = 0; i < N; ++i) {
    rx[i] = ay[i] * bz[i] - az[i] * by[i];
    ry[i] = az[i] * bx[i] - ax[i] * bz[i];
    rz[i] = ax[i] * by[i] - ay[i] * bx[i];
  }

  return Vector3SoA<T, N>(
      Vec<T, N>::load(rx.data()),
      Vec<T, N>::load(ry.data()),
      Vec<T, N>::load(rz.data()));
}

/// Reference B -- "fused scalar": explicit std::fma, matching the fusion
/// policy dart::simd::cross()'s SoA batch kernel already applies. IEEE-754
/// (2008) requires fma() to be a correctly-rounded, single-rounding fused
/// multiply-add, so this reference is bit-identical to any conforming
/// hardware FMA instruction (SSE4.2 has none; AVX2/FMA3, AVX-512, and NEON's
/// FMA all qualify) for the same operand bit patterns -- which is exactly
/// what the D1 "consistent FMA usage" clause needs to hold across backends.
template <typename T, std::size_t N>
[[nodiscard]] Vector3SoA<T, N> scalarCrossFused(
    const Vector3SoA<T, N>& a, const Vector3SoA<T, N>& b) noexcept
{
  std::array<T, N> ax{}, ay{}, az{}, bx{}, by{}, bz{}, rx{}, ry{}, rz{};
  a.x.store(ax.data());
  a.y.store(ay.data());
  a.z.store(az.data());
  b.x.store(bx.data());
  b.y.store(by.data());
  b.z.store(bz.data());

  for (std::size_t i = 0; i < N; ++i) {
    rx[i] = std::fma(ay[i], bz[i], -(az[i] * by[i]));
    ry[i] = std::fma(az[i], bx[i], -(ax[i] * bz[i]));
    rz[i] = std::fma(ax[i], by[i], -(ay[i] * bx[i]));
  }

  return Vector3SoA<T, N>(
      Vec<T, N>::load(rx.data()),
      Vec<T, N>::load(ry.data()),
      Vec<T, N>::load(rz.data()));
}

/// Bit-for-bit comparator (not a tolerance/epsilon comparator): the D1
/// contract is "bit-identical", so the check must be memcmp, not
/// approximate. -0.0 vs 0.0 intentionally compares unequal here.
template <typename T, std::size_t N>
[[nodiscard]] bool bitwiseEqual(
    const Vector3SoA<T, N>& a, const Vector3SoA<T, N>& b) noexcept
{
  std::array<T, N> ax{}, ay{}, az{}, bx{}, by{}, bz{};
  a.x.store(ax.data());
  a.y.store(ay.data());
  a.z.store(az.data());
  b.x.store(bx.data());
  b.y.store(by.data());
  b.z.store(bz.data());

  return std::memcmp(ax.data(), bx.data(), sizeof(T) * N) == 0
         && std::memcmp(ay.data(), by.data(), sizeof(T) * N) == 0
         && std::memcmp(az.data(), bz.data(), sizeof(T) * N) == 0;
}

struct Cross3BatchCheckResult
{
  bool matchesFusedReference;
  bool matchesNaiveReference;
};

/// Fixed, deterministic 4-wide input batch (no RNG: reproducible across
/// hosts/runs/backends by construction) exercising the SoA batch cross
/// product against both scalar references.
///
/// Expected outcome once this is promoted to a gtest case under
/// ci_simd.yml's scalar/sse42/avx/avx2 matrix (WP-PG.41 scope, not this
/// packet):
///   - matchesFusedReference: true on every backend. dart::simd::cross()'s
///     fmsub() and scalarCrossFused()'s std::fma() are both correctly-
///     rounded single-rounding FMA per IEEE-754, so they agree bit-for-bit
///     regardless of which backend actually executes the instruction.
///   - matchesNaiveReference: NOT guaranteed true anywhere the fusion
///     actually changes the rounding (i.e. whenever the two products do not
///     happen to be exactly representable). This is the concrete case for
///     D1's coding rule: the existing unfused AoS `Vector3<T>::cross()`
///     formula must not be treated as "the" scalar reference once WP-PG.41
///     mixes AoS and SoA cross products on the same state-affecting path --
///     doing so would reintroduce exactly the drift D1 is meant to close.
template <typename T = double, std::size_t N = 4>
[[nodiscard]] Cross3BatchCheckResult runCross3BatchBitEqualityCheck() noexcept
{
  const Vector3SoA<T, N> a(
      Vec<T, N>::set(
          T(1.0000001), T(-2.5), T(3.3333333333), T(0.0001)),
      Vec<T, N>::set(
          T(4.5), T(5.0000002), T(-6.6666666), T(123456.789)),
      Vec<T, N>::set(
          T(-7.25), T(8.1), T(9.0000003), T(-0.00042)));
  const Vector3SoA<T, N> b(
      Vec<T, N>::set(T(0.5), T(1.1), T(-2.2222222), T(42.0)),
      Vec<T, N>::set(T(-1.5), T(2.2), T(3.3333333), T(-0.0007)),
      Vec<T, N>::set(T(2.5), T(-3.3), T(4.4444444), T(8675309.0)));

  const Vector3SoA<T, N> simdResult = dart::simd::cross(a, b);
  const Vector3SoA<T, N> naiveResult = scalarCrossNaive(a, b);
  const Vector3SoA<T, N> fusedResult = scalarCrossFused(a, b);

  return Cross3BatchCheckResult{
      bitwiseEqual(simdResult, fusedResult),
      bitwiseEqual(simdResult, naiveResult)};
}

} // namespace wp_pg_40
