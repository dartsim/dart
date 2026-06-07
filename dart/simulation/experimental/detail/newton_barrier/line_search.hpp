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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *   TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 *   THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *   SUCH DAMAGE.
 */

#pragma once

#include <cstddef>

namespace dart::simulation::experimental::detail::newton_barrier {

struct LineSearchOptions
{
  double minSeparation = 0.0;
  double tolerance = 1e-6;
  int maxIterations = 64;
};

struct LineSearchStats
{
  std::size_t pointPointChecks = 0;
  std::size_t pointEdgeChecks = 0;
  std::size_t edgeEdgeChecks = 0;
  std::size_t pointTriangleChecks = 0;
  std::size_t hits = 0;
  std::size_t misses = 0;
  std::size_t indeterminate = 0;
  std::size_t zeroStepCount = 0;
};

inline void accumulateLineSearchStats(
    LineSearchStats& total, const LineSearchStats& addend)
{
  total.pointPointChecks += addend.pointPointChecks;
  total.pointEdgeChecks += addend.pointEdgeChecks;
  total.edgeEdgeChecks += addend.edgeEdgeChecks;
  total.pointTriangleChecks += addend.pointTriangleChecks;
  total.hits += addend.hits;
  total.misses += addend.misses;
  total.indeterminate += addend.indeterminate;
  total.zeroStepCount += addend.zeroStepCount;
}

} // namespace dart::simulation::experimental::detail::newton_barrier
