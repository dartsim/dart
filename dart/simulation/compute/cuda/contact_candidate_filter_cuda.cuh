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

struct PointTriangleContactStencil
{
  std::uint32_t point = 0;
  std::uint32_t triangle = 0;
};

struct EdgeEdgeContactStencil
{
  std::uint32_t edgeAStart = 0;
  std::uint32_t edgeAEnd = 0;
  std::uint32_t edgeBStart = 0;
  std::uint32_t edgeBEnd = 0;
};

struct ContactCandidateFilterTiming
{
  double setupNs = 0.0;
  double hostToDeviceNs = 0.0;
  double kernelNs = 0.0;
  double deviceToHostNs = 0.0;
};

struct PointTriangleCandidateFilterResult
{
  std::vector<double> squaredDistances;
  std::vector<std::uint8_t> accepted;
  std::size_t acceptedCount = 0;
  double maxAcceptedSquaredDistance = 0.0;
  ContactCandidateFilterTiming timing;
};

struct EdgeEdgeCandidateFilterResult
{
  std::vector<double> squaredDistances;
  std::vector<std::uint8_t> accepted;
  std::size_t acceptedCount = 0;
  double maxAcceptedSquaredDistance = 0.0;
  ContactCandidateFilterTiming timing;
};

/// Filter preassembled point-triangle contact stencils on CUDA.
///
/// This is private evidence for the PLAN-083 GPU packet: it compares the same
/// deterministic point/triangle stencil list against the CPU candidate path.
/// It does not build broad-phase AABBs, edge-edge candidates, or scene-level
/// solver state.
void filterPointTriangleContactStencilsCuda(
    const std::vector<double>& positions,
    const std::vector<std::uint32_t>& triangleIndices,
    const std::vector<PointTriangleContactStencil>& stencils,
    double activationDistance,
    PointTriangleCandidateFilterResult& result);

/// Filter preassembled edge-edge contact stencils on CUDA.
///
/// This is private evidence for the PLAN-083 GPU packet: it extends the
/// deterministic primitive-stencil parity path beyond point-triangle rows. It
/// does not build broad-phase AABBs or scene-level solver state.
void filterEdgeEdgeContactStencilsCuda(
    const std::vector<double>& positions,
    const std::vector<EdgeEdgeContactStencil>& stencils,
    double activationDistance,
    EdgeEdgeCandidateFilterResult& result);

} // namespace dart::simulation::compute::cuda
