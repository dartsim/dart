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

// WP-PG.40 prototype harness. Compile-only in this packet (see
// 08-simd-contracts.md); the "council of one TU per backend" pattern below
// mirrors what ci_simd.yml's DART_SIMD_FORCE_SCALAR / sse42 / avx / avx2
// matrix already does at the CMake-target level (dart/simd/config.hpp
// backend selection is compile-time), so this file compiles and links
// standalone under any of those flag sets without modification.
//
// Verified in this packet with (single logical command, wrapped here only
// for line length -- no trailing backslashes, to avoid -Wcomment):
//   g++ -std=c++17 -fsyntax-only -Wall -Wextra -Werror
//     -I <worktree-root> -I <eigen3-include>
//     wp_pg_40_cross3_batch_test.cpp                          # (a) default
//   ... -DDART_SIMD_FORCE_SCALAR ...                          # (b) scalar
//   ... -mavx2 -mfma ...                                       # (c) avx2
//
// Promoting this to an executed UNIT_simd_* gtest case (so the bit-equality
// assertions actually run under ci_simd.yml) is WP-PG.41 scope, once the
// dart/math seam exists to host it.

#include "wp_pg_40_cross3_batch.hpp"

#include <cstdio>

int main()
{
  const wp_pg_40::Cross3BatchCheckResult result
      = wp_pg_40::runCross3BatchBitEqualityCheck<double, 4>();

  std::printf(
      "[wp-pg-40] backend=%s matches_fused_reference=%d "
      "matches_naive_reference=%d\n",
      dart::simd::backend_name,
      result.matchesFusedReference ? 1 : 0,
      result.matchesNaiveReference ? 1 : 0);

  // D1 requires the FMA-consistent reference to match on every backend; it
  // does not require the naive (unfused) reference to match (see the header
  // comment). Only the fused comparison failing indicates an actual bug
  // (a backend's fmsub() not being a correctly-rounded IEEE-754 FMA).
  return result.matchesFusedReference ? 0 : 1;
}
