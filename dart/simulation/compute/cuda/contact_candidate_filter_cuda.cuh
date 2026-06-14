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

namespace detail {

struct ContactCandidateSweepAabbItem
{
  double minX = 0.0;
  double minY = 0.0;
  double minZ = 0.0;
  double maxX = 0.0;
  double maxY = 0.0;
  double maxZ = 0.0;
  std::uint32_t id = 0;
};

} // namespace detail

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

struct PointTriangleCandidateBuildResult
{
  std::vector<double> squaredDistances;
  std::vector<std::uint8_t> accepted;
  std::vector<std::uint32_t> acceptedPointIndices;
  std::vector<std::uint32_t> acceptedTriangleIndices;
  std::vector<double> acceptedSquaredDistances;
  std::size_t pointCount = 0;
  std::size_t triangleCount = 0;
  std::size_t pairCount = 0;
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

struct EdgeEdgeCandidateBuildResult
{
  std::vector<double> squaredDistances;
  std::vector<std::uint8_t> accepted;
  std::vector<std::uint32_t> acceptedEdgeAIndices;
  std::vector<std::uint32_t> acceptedEdgeBIndices;
  std::vector<double> acceptedSquaredDistances;
  std::size_t edgeCount = 0;
  std::size_t pairCount = 0;
  std::size_t acceptedCount = 0;
  double maxAcceptedSquaredDistance = 0.0;
  ContactCandidateFilterTiming timing;
};

struct SweptPointTriangleCandidateBuildResult
{
  std::vector<double> endpointSquaredDistances;
  std::vector<std::uint8_t> accepted;
  std::vector<std::uint32_t> acceptedPointIndices;
  std::vector<std::uint32_t> acceptedTriangleIndices;
  std::vector<double> acceptedEndpointSquaredDistances;
  std::size_t pointCount = 0;
  std::size_t triangleCount = 0;
  std::size_t pairCount = 0;
  std::size_t acceptedCount = 0;
  double maxAcceptedEndpointSquaredDistance = 0.0;
  ContactCandidateFilterTiming timing;
};

struct SweptEdgeEdgeCandidateBuildResult
{
  std::vector<double> endpointSquaredDistances;
  std::vector<std::uint8_t> accepted;
  std::vector<std::uint32_t> acceptedEdgeAIndices;
  std::vector<std::uint32_t> acceptedEdgeBIndices;
  std::vector<double> acceptedEndpointSquaredDistances;
  std::size_t edgeCount = 0;
  std::size_t pairCount = 0;
  std::size_t acceptedCount = 0;
  double maxAcceptedEndpointSquaredDistance = 0.0;
  ContactCandidateFilterTiming timing;
};

struct SweptPointTriangleSweepResult
{
  std::vector<std::uint32_t> acceptedPointIndices;
  std::vector<std::uint32_t> acceptedTriangleIndices;
  std::vector<double> acceptedEndpointSquaredDistances;
  std::size_t pointCount = 0;
  std::size_t triangleCount = 0;
  std::size_t pairCapacity = 0;
  std::size_t acceptedCount = 0;
  double maxAcceptedEndpointSquaredDistance = 0.0;
  ContactCandidateFilterTiming timing;
};

struct SweptEdgeEdgeSweepResult
{
  std::vector<std::uint32_t> acceptedEdgeAIndices;
  std::vector<std::uint32_t> acceptedEdgeBIndices;
  std::vector<double> acceptedEndpointSquaredDistances;
  std::size_t edgeCount = 0;
  std::size_t pairCapacity = 0;
  std::size_t acceptedCount = 0;
  double maxAcceptedEndpointSquaredDistance = 0.0;
  ContactCandidateFilterTiming timing;
};

struct SweptPointTriangleCandidateBufferResult
{
  std::vector<double> endpointSquaredDistances;
  std::size_t pointCount = 0;
  std::size_t triangleCount = 0;
  std::size_t pairCount = 0;
  double maxEndpointSquaredDistance = 0.0;
  ContactCandidateFilterTiming timing;
};

struct SweptEdgeEdgeCandidateBufferResult
{
  std::vector<double> endpointSquaredDistances;
  std::size_t edgeCount = 0;
  std::size_t pairCount = 0;
  double maxEndpointSquaredDistance = 0.0;
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

/// Build a private all-pairs point-triangle candidate mask on CUDA.
///
/// This packet evaluates every supplied candidate point against every supplied
/// triangle and writes a deterministic accepted mask plus a compact accepted
/// point/triangle list produced on the device before readback. Incident
/// point-triangle pairs are masked out. It is stronger than a preassembled
/// stencil filter, but it intentionally does not cover sweep broad-phase
/// pruning, scene-level solver state, or a public GPU solver backend.
void buildPointTriangleContactCandidateMaskCuda(
    const std::vector<double>& positions,
    const std::vector<std::uint32_t>& pointIndices,
    const std::vector<std::uint32_t>& triangleIndices,
    double activationDistance,
    PointTriangleCandidateBuildResult& result);

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

/// Build a private all-pairs edge-edge candidate mask on CUDA.
///
/// This packet evaluates every supplied edge slot against every supplied edge
/// slot, masks self/duplicate/incident edge pairs, and writes a deterministic
/// accepted mask plus compact accepted edge-slot pairs produced on the device
/// before readback. It intentionally does not cover sweep broad-phase pruning,
/// scene-level solver state, or a public GPU solver backend.
void buildEdgeEdgeContactCandidateMaskCuda(
    const std::vector<double>& positions,
    const std::vector<std::uint32_t>& edgeIndices,
    double activationDistance,
    EdgeEdgeCandidateBuildResult& result);

/// Build a private motion-aware point-triangle candidate list on CUDA.
///
/// This packet evaluates swept AABB overlap for every supplied point slot
/// against every supplied triangle, masks incident pairs, and compacts accepted
/// point/triangle pairs on the device. The endpoint distance metadata matches
/// the CPU motion-aware candidate builders. This is closer to runtime candidate
/// construction than a static all-pairs distance mask, but it still does not
/// cover sweep-and-prune sorting, scene-owned buffers, or a public GPU solver
/// backend.
void buildSweptPointTriangleContactCandidateMaskCuda(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    const std::vector<std::uint32_t>& pointIndices,
    const std::vector<std::uint32_t>& triangleIndices,
    double activationDistance,
    SweptPointTriangleCandidateBuildResult& result);

/// Build a private motion-aware edge-edge candidate list on CUDA.
///
/// This packet evaluates swept AABB overlap for every supplied edge-slot pair,
/// masks self/duplicate/incident pairs, and compacts accepted edge-slot pairs
/// on the device. It intentionally stays private packet evidence rather than a
/// runtime broad-phase or public GPU backend.
void buildSweptEdgeEdgeContactCandidateMaskCuda(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    const std::vector<std::uint32_t>& edgeIndices,
    double activationDistance,
    SweptEdgeEdgeCandidateBuildResult& result);

/// Build a private motion-aware point-triangle sweep-and-prune list on CUDA.
///
/// This packet constructs swept primitive AABBs, sorts them on the device by
/// the CPU sweep key, and scatters compact overlap candidates in deterministic
/// sweep order. It remains private packet evidence: it does not own scene
/// buffers, runtime solver integration, or a public GPU backend.
void buildSweptPointTriangleSweepCuda(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    const std::vector<std::uint32_t>& pointIndices,
    const std::vector<std::uint32_t>& triangleIndices,
    double activationDistance,
    SweptPointTriangleSweepResult& result);

/// Build a private motion-aware edge-edge sweep-and-prune list on CUDA.
///
/// This packet mirrors the CPU self-sweep ordering for compact surface-edge
/// slots after device-side swept-AABB construction and sorting. It is still a
/// benchmark packet, not a runtime GPU candidate-buffer owner.
void buildSweptEdgeEdgeSweepCuda(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    const std::vector<std::uint32_t>& edgeIndices,
    double activationDistance,
    SweptEdgeEdgeSweepResult& result);

/// Evaluate a compact runtime point-triangle candidate buffer on CUDA.
///
/// This packet consumes candidate keys produced by the CPU motion-aware sweep
/// builder and recomputes the endpoint-distance metadata on the GPU. It proves
/// private runtime-buffer parity without claiming GPU sweep-and-prune sorting,
/// scene-owned buffers, or a public GPU solver backend.
void evaluateSweptPointTriangleCandidateBufferCuda(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    const std::vector<std::uint32_t>& triangleIndices,
    const std::vector<std::uint32_t>& candidatePointIndices,
    const std::vector<std::uint32_t>& candidateTriangleIndices,
    SweptPointTriangleCandidateBufferResult& result);

/// Evaluate a compact runtime edge-edge candidate buffer on CUDA.
///
/// This packet consumes edge-slot candidate keys produced by the CPU
/// motion-aware sweep builder and recomputes the endpoint-distance metadata on
/// the GPU. It intentionally stays private packet evidence rather than a
/// runtime broad-phase or public GPU backend.
void evaluateSweptEdgeEdgeCandidateBufferCuda(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    const std::vector<std::uint32_t>& edgeIndices,
    const std::vector<std::uint32_t>& candidateEdgeAIndices,
    const std::vector<std::uint32_t>& candidateEdgeBIndices,
    SweptEdgeEdgeCandidateBufferResult& result);

} // namespace dart::simulation::compute::cuda
