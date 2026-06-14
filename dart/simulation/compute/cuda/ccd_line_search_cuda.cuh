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
 *     copyright notice, this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::compute::cuda {

struct PointTriangleCcdLineSearchPair
{
  double pointStart[3]{};
  double pointEnd[3]{};
  double triangleAStart[3]{};
  double triangleAEnd[3]{};
  double triangleBStart[3]{};
  double triangleBEnd[3]{};
  double triangleCStart[3]{};
  double triangleCEnd[3]{};
};

struct EdgeEdgeCcdLineSearchPair
{
  double edgeA0Start[3]{};
  double edgeA0End[3]{};
  double edgeA1Start[3]{};
  double edgeA1End[3]{};
  double edgeB0Start[3]{};
  double edgeB0End[3]{};
  double edgeB1Start[3]{};
  double edgeB1End[3]{};
};

struct CcdLineSearchOptions
{
  double minSeparation = 0.0;
  double tolerance = 1e-6;
  int maxIterations = 64;
};

struct CcdLineSearchTiming
{
  double setupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double kernelNs = 0.0;
  double deviceToHostNs = 0.0;
};

struct CcdLineSearchResult
{
  std::vector<double> stepBounds;
  std::vector<std::uint8_t> hits;
  std::vector<std::uint8_t> indeterminate;
  std::size_t hitCount = 0;
  double minStepBound = 1.0;
  CcdLineSearchTiming timing;
};

using PointTriangleCcdLineSearchResult = CcdLineSearchResult;
using EdgeEdgeCcdLineSearchResult = CcdLineSearchResult;

/// Evaluate a private endpoint-linear point-triangle CCD line-search packet.
///
/// The CUDA packet covers linear point and triangle vertex trajectories through
/// the same additive conservative-advancement contract as the CPU helper. It is
/// private staged GPU evidence, not a public CCD backend.
void evaluatePointTriangleCcdLineSearchCuda(
    const std::vector<PointTriangleCcdLineSearchPair>& pairs,
    const CcdLineSearchOptions& options,
    PointTriangleCcdLineSearchResult& result);

/// Evaluate a private endpoint-linear edge-edge CCD line-search packet.
///
/// The CUDA packet covers independently moving segment endpoints and matches
/// the internal CPU additive conservative-advancement contract for reduced
/// edge-edge fixtures. It is private staged GPU evidence, not a public CCD
/// backend.
void evaluateEdgeEdgeCcdLineSearchCuda(
    const std::vector<EdgeEdgeCcdLineSearchPair>& pairs,
    const CcdLineSearchOptions& options,
    EdgeEdgeCcdLineSearchResult& result);

} // namespace dart::simulation::compute::cuda
