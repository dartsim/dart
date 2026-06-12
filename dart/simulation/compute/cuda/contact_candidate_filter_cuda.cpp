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

#include <dart/simulation/common/exceptions.hpp>

#include <cuda_runtime_api.h>
#include <dart/simulation/compute/cuda/contact_candidate_filter_cuda.cuh>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/device_buffer.cuh>

#include <algorithm>
#include <chrono>
#include <string_view>

namespace sx = dart::simulation;

namespace dart::simulation::compute::cuda {
namespace detail {

cudaError_t launchPointTriangleContactStencilFilterKernel(
    const double* positions,
    const std::uint32_t* triangleIndices,
    const PointTriangleContactStencil* stencils,
    double activationDistance,
    double* squaredDistances,
    std::uint8_t* accepted,
    std::size_t stencilCount);

cudaError_t launchPointTriangleContactCandidateMaskKernel(
    const double* positions,
    const std::uint32_t* pointIndices,
    const std::uint32_t* triangleIndices,
    double activationDistance,
    double* squaredDistances,
    std::uint8_t* accepted,
    std::uint32_t* acceptedPointIndices,
    std::uint32_t* acceptedTriangleIndices,
    double* acceptedSquaredDistances,
    std::uint32_t* compactedCount,
    std::uint32_t* acceptedBlockCounts,
    std::uint32_t* acceptedBlockOffsets,
    std::size_t pointCount,
    std::size_t triangleCount);

cudaError_t launchEdgeEdgeContactStencilFilterKernel(
    const double* positions,
    const EdgeEdgeContactStencil* stencils,
    double activationDistance,
    double* squaredDistances,
    std::uint8_t* accepted,
    std::size_t stencilCount);

cudaError_t launchEdgeEdgeContactCandidateMaskKernel(
    const double* positions,
    const std::uint32_t* edgeIndices,
    double activationDistance,
    double* squaredDistances,
    std::uint8_t* accepted,
    std::uint32_t* acceptedEdgeAIndices,
    std::uint32_t* acceptedEdgeBIndices,
    double* acceptedSquaredDistances,
    std::uint32_t* compactedCount,
    std::uint32_t* acceptedBlockCounts,
    std::uint32_t* acceptedBlockOffsets,
    std::size_t edgeCount);

cudaError_t launchSweptPointTriangleContactCandidateMaskKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* pointIndices,
    const std::uint32_t* triangleIndices,
    double activationDistance,
    double* endpointSquaredDistances,
    std::uint8_t* accepted,
    std::uint32_t* acceptedPointIndices,
    std::uint32_t* acceptedTriangleIndices,
    double* acceptedEndpointSquaredDistances,
    std::uint32_t* compactedCount,
    std::uint32_t* acceptedBlockCounts,
    std::uint32_t* acceptedBlockOffsets,
    std::size_t pointCount,
    std::size_t triangleCount);

cudaError_t launchSweptEdgeEdgeContactCandidateMaskKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* edgeIndices,
    double activationDistance,
    double* endpointSquaredDistances,
    std::uint8_t* accepted,
    std::uint32_t* acceptedEdgeAIndices,
    std::uint32_t* acceptedEdgeBIndices,
    double* acceptedEndpointSquaredDistances,
    std::uint32_t* compactedCount,
    std::uint32_t* acceptedBlockCounts,
    std::uint32_t* acceptedBlockOffsets,
    std::size_t edgeCount);

cudaError_t launchSweptPointTriangleSweepKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* pointIndices,
    const std::uint32_t* triangleIndices,
    double activationDistance,
    ContactCandidateSweepAabbItem* pointItems,
    ContactCandidateSweepAabbItem* triangleItems,
    std::uint32_t* pairCounts,
    std::uint32_t* pairOffsets,
    std::uint32_t* compactedCount,
    std::uint32_t* acceptedPointIndices,
    std::uint32_t* acceptedTriangleIndices,
    double* acceptedEndpointSquaredDistances,
    std::size_t pointCount,
    std::size_t triangleCount,
    std::size_t paddedPointCount,
    std::size_t paddedTriangleCount);

cudaError_t launchSweptEdgeEdgeSweepKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* edgeIndices,
    double activationDistance,
    ContactCandidateSweepAabbItem* edgeItems,
    std::uint32_t* pairCounts,
    std::uint32_t* pairOffsets,
    std::uint32_t* compactedCount,
    std::uint32_t* acceptedEdgeAIndices,
    std::uint32_t* acceptedEdgeBIndices,
    double* acceptedEndpointSquaredDistances,
    std::size_t edgeCount,
    std::size_t paddedEdgeCount);

cudaError_t launchSweptPointTriangleCandidateBufferKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* triangleIndices,
    const std::uint32_t* candidatePointIndices,
    const std::uint32_t* candidateTriangleIndices,
    double* endpointSquaredDistances,
    std::size_t pairCount);

cudaError_t launchSweptEdgeEdgeCandidateBufferKernel(
    const double* startPositions,
    const double* endPositions,
    const std::uint32_t* edgeIndices,
    const std::uint32_t* candidateEdgeAIndices,
    const std::uint32_t* candidateEdgeBIndices,
    double* endpointSquaredDistances,
    std::size_t pairCount);

} // namespace detail
namespace {

using Clock = std::chrono::steady_clock;

constexpr std::size_t kContactCandidateMaskBlockSize = 256u;

double elapsedNs(const Clock::time_point start, const Clock::time_point end)
{
  return static_cast<double>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
          .count());
}

std::size_t nextPowerOfTwo(std::size_t value)
{
  std::size_t result = 1u;
  while (result < value) {
    result <<= 1u;
  }
  return result;
}

void throwIfCudaRuntimeUnavailable()
{
  DART_SIMULATION_THROW_T_IF(
      !isCudaRuntimeAvailable(),
      sx::InvalidOperationException,
      "CUDA runtime has no available device");
}

std::size_t validateInputs(
    const std::vector<double>& positions,
    const std::vector<std::uint32_t>& triangleIndices,
    const std::vector<PointTriangleContactStencil>& stencils,
    std::string_view operation)
{
  DART_SIMULATION_THROW_T_IF(
      positions.size() % 3 != 0,
      sx::InvalidArgumentException,
      "{} expects xyz-packed positions but received {} doubles",
      operation,
      positions.size());
  DART_SIMULATION_THROW_T_IF(
      triangleIndices.size() % 3 != 0,
      sx::InvalidArgumentException,
      "{} expects triplet-packed triangles but received {} indices",
      operation,
      triangleIndices.size());

  const std::size_t pointCount = positions.size() / 3;
  const std::size_t triangleCount = triangleIndices.size() / 3;
  for (const auto& stencil : stencils) {
    DART_SIMULATION_THROW_T_IF(
        stencil.point >= pointCount,
        sx::InvalidArgumentException,
        "{} stencil point {} is outside {} positions",
        operation,
        stencil.point,
        pointCount);
    DART_SIMULATION_THROW_T_IF(
        stencil.triangle >= triangleCount,
        sx::InvalidArgumentException,
        "{} stencil triangle {} is outside {} triangles",
        operation,
        stencil.triangle,
        triangleCount);
  }
  for (const std::uint32_t index : triangleIndices) {
    DART_SIMULATION_THROW_T_IF(
        index >= pointCount,
        sx::InvalidArgumentException,
        "{} triangle index {} is outside {} positions",
        operation,
        index,
        pointCount);
  }

  return pointCount;
}

std::size_t validateInputs(
    const std::vector<double>& positions,
    const std::vector<EdgeEdgeContactStencil>& stencils,
    std::string_view operation)
{
  DART_SIMULATION_THROW_T_IF(
      positions.size() % 3 != 0,
      sx::InvalidArgumentException,
      "{} expects xyz-packed positions but received {} doubles",
      operation,
      positions.size());

  const std::size_t pointCount = positions.size() / 3;
  for (const auto& stencil : stencils) {
    DART_SIMULATION_THROW_T_IF(
        stencil.edgeAStart >= pointCount || stencil.edgeAEnd >= pointCount
            || stencil.edgeBStart >= pointCount
            || stencil.edgeBEnd >= pointCount,
        sx::InvalidArgumentException,
        "{} edge-edge stencil references positions outside {} points",
        operation,
        pointCount);
  }

  return pointCount;
}

std::size_t validateCandidateMaskInputs(
    const std::vector<double>& positions,
    const std::vector<std::uint32_t>& pointIndices,
    const std::vector<std::uint32_t>& triangleIndices,
    std::string_view operation)
{
  DART_SIMULATION_THROW_T_IF(
      positions.size() % 3 != 0,
      sx::InvalidArgumentException,
      "{} expects xyz-packed positions but received {} doubles",
      operation,
      positions.size());
  DART_SIMULATION_THROW_T_IF(
      triangleIndices.size() % 3 != 0,
      sx::InvalidArgumentException,
      "{} expects triplet-packed triangles but received {} indices",
      operation,
      triangleIndices.size());

  const std::size_t pointCount = positions.size() / 3;
  for (const std::uint32_t point : pointIndices) {
    DART_SIMULATION_THROW_T_IF(
        point >= pointCount,
        sx::InvalidArgumentException,
        "{} candidate point {} is outside {} positions",
        operation,
        point,
        pointCount);
  }
  for (const std::uint32_t index : triangleIndices) {
    DART_SIMULATION_THROW_T_IF(
        index >= pointCount,
        sx::InvalidArgumentException,
        "{} triangle index {} is outside {} positions",
        operation,
        index,
        pointCount);
  }

  return pointCount;
}

std::size_t validateEdgeEdgeCandidateMaskInputs(
    const std::vector<double>& positions,
    const std::vector<std::uint32_t>& edgeIndices,
    std::string_view operation)
{
  DART_SIMULATION_THROW_T_IF(
      positions.size() % 3 != 0,
      sx::InvalidArgumentException,
      "{} expects xyz-packed positions but received {} doubles",
      operation,
      positions.size());
  DART_SIMULATION_THROW_T_IF(
      edgeIndices.size() % 2 != 0,
      sx::InvalidArgumentException,
      "{} expects pair-packed edges but received {} indices",
      operation,
      edgeIndices.size());

  const std::size_t pointCount = positions.size() / 3;
  for (const std::uint32_t index : edgeIndices) {
    DART_SIMULATION_THROW_T_IF(
        index >= pointCount,
        sx::InvalidArgumentException,
        "{} edge index {} is outside {} positions",
        operation,
        index,
        pointCount);
  }

  return pointCount;
}

void validateMatchingEndPositions(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    std::string_view operation)
{
  DART_SIMULATION_THROW_T_IF(
      startPositions.size() != endPositions.size(),
      sx::InvalidArgumentException,
      "{} expects matching start/end xyz-packed positions but received {} and "
      "{} doubles",
      operation,
      startPositions.size(),
      endPositions.size());
}

void validatePointTriangleCandidateBufferInputs(
    const std::vector<double>& startPositions,
    const std::vector<std::uint32_t>& triangleIndices,
    const std::vector<std::uint32_t>& candidatePointIndices,
    const std::vector<std::uint32_t>& candidateTriangleIndices,
    std::string_view operation)
{
  validateCandidateMaskInputs(
      startPositions, candidatePointIndices, triangleIndices, operation);
  DART_SIMULATION_THROW_T_IF(
      candidatePointIndices.size() != candidateTriangleIndices.size(),
      sx::InvalidArgumentException,
      "{} expects matching point/triangle candidate buffers but received {} "
      "and {} entries",
      operation,
      candidatePointIndices.size(),
      candidateTriangleIndices.size());

  const std::size_t triangleCount = triangleIndices.size() / 3u;
  for (const std::uint32_t triangle : candidateTriangleIndices) {
    DART_SIMULATION_THROW_T_IF(
        triangle >= triangleCount,
        sx::InvalidArgumentException,
        "{} candidate triangle {} is outside {} triangles",
        operation,
        triangle,
        triangleCount);
  }
}

void validateEdgeEdgeCandidateBufferInputs(
    const std::vector<double>& startPositions,
    const std::vector<std::uint32_t>& edgeIndices,
    const std::vector<std::uint32_t>& candidateEdgeAIndices,
    const std::vector<std::uint32_t>& candidateEdgeBIndices,
    std::string_view operation)
{
  validateEdgeEdgeCandidateMaskInputs(startPositions, edgeIndices, operation);
  DART_SIMULATION_THROW_T_IF(
      candidateEdgeAIndices.size() != candidateEdgeBIndices.size(),
      sx::InvalidArgumentException,
      "{} expects matching edge candidate buffers but received {} and {} "
      "entries",
      operation,
      candidateEdgeAIndices.size(),
      candidateEdgeBIndices.size());

  const std::size_t edgeCount = edgeIndices.size() / 2u;
  for (const std::uint32_t edge : candidateEdgeAIndices) {
    DART_SIMULATION_THROW_T_IF(
        edge >= edgeCount,
        sx::InvalidArgumentException,
        "{} candidate edge-a {} is outside {} edges",
        operation,
        edge,
        edgeCount);
  }
  for (const std::uint32_t edge : candidateEdgeBIndices) {
    DART_SIMULATION_THROW_T_IF(
        edge >= edgeCount,
        sx::InvalidArgumentException,
        "{} candidate edge-b {} is outside {} edges",
        operation,
        edge,
        edgeCount);
  }
}

} // namespace

//==============================================================================
void filterPointTriangleContactStencilsCuda(
    const std::vector<double>& positions,
    const std::vector<std::uint32_t>& triangleIndices,
    const std::vector<PointTriangleContactStencil>& stencils,
    double activationDistance,
    PointTriangleCandidateFilterResult& result)
{
  const auto setupStart = Clock::now();
  static constexpr std::string_view kOperation
      = "filterPointTriangleContactStencilsCuda";
  validateInputs(positions, triangleIndices, stencils, kOperation);

  result = PointTriangleCandidateFilterResult{};
  result.squaredDistances.resize(stencils.size());
  result.accepted.resize(stencils.size());

  if (stencils.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> devicePositions(positions.size());
  DeviceBuffer<std::uint32_t> deviceTriangles(triangleIndices.size());
  DeviceBuffer<PointTriangleContactStencil> deviceStencils(stencils.size());
  DeviceBuffer<double> deviceSquaredDistances(stencils.size());
  DeviceBuffer<std::uint8_t> deviceAccepted(stencils.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  devicePositions.copyToDevice(positions, "contact positions copy");
  deviceTriangles.copyToDevice(triangleIndices, "contact triangles copy");
  deviceStencils.copyToDevice(stencils, "contact stencils copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchPointTriangleContactStencilFilterKernel(
          devicePositions.data(),
          deviceTriangles.data(),
          deviceStencils.data(),
          std::max(0.0, activationDistance),
          deviceSquaredDistances.data(),
          deviceAccepted.data(),
          stencils.size()),
      "contact candidate filter kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "contact candidate filter synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceSquaredDistances.copyFromDevice(
      result.squaredDistances, "contact squared distances copy");
  deviceAccepted.copyFromDevice(result.accepted, "contact accepted copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (std::size_t i = 0; i < result.accepted.size(); ++i) {
    if (result.accepted[i] != 0) {
      ++result.acceptedCount;
      result.maxAcceptedSquaredDistance = std::max(
          result.maxAcceptedSquaredDistance, result.squaredDistances[i]);
    }
  }
}

//==============================================================================
void buildPointTriangleContactCandidateMaskCuda(
    const std::vector<double>& positions,
    const std::vector<std::uint32_t>& pointIndices,
    const std::vector<std::uint32_t>& triangleIndices,
    double activationDistance,
    PointTriangleCandidateBuildResult& result)
{
  const auto setupStart = Clock::now();
  static constexpr std::string_view kOperation
      = "buildPointTriangleContactCandidateMaskCuda";
  validateCandidateMaskInputs(
      positions, pointIndices, triangleIndices, kOperation);

  result = PointTriangleCandidateBuildResult{};
  result.pointCount = pointIndices.size();
  result.triangleCount = triangleIndices.size() / 3u;
  result.pairCount = result.pointCount * result.triangleCount;
  result.squaredDistances.resize(result.pairCount);
  result.accepted.resize(result.pairCount);

  if (result.pairCount == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> devicePositions(positions.size());
  DeviceBuffer<std::uint32_t> devicePoints(pointIndices.size());
  DeviceBuffer<std::uint32_t> deviceTriangles(triangleIndices.size());
  DeviceBuffer<double> deviceSquaredDistances(result.pairCount);
  DeviceBuffer<std::uint8_t> deviceAccepted(result.pairCount);
  DeviceBuffer<std::uint32_t> deviceAcceptedPointIndices(result.pairCount);
  DeviceBuffer<std::uint32_t> deviceAcceptedTriangleIndices(result.pairCount);
  DeviceBuffer<double> deviceAcceptedSquaredDistances(result.pairCount);
  DeviceBuffer<std::uint32_t> deviceCompactedCount(1u);
  const std::size_t compactBlockCount
      = (result.pairCount + kContactCandidateMaskBlockSize - 1u)
        / kContactCandidateMaskBlockSize;
  DeviceBuffer<std::uint32_t> deviceAcceptedBlockCounts(compactBlockCount);
  DeviceBuffer<std::uint32_t> deviceAcceptedBlockOffsets(compactBlockCount);
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  devicePositions.copyToDevice(positions, "contact candidate positions copy");
  devicePoints.copyToDevice(
      pointIndices, "contact candidate point indices copy");
  deviceTriangles.copyToDevice(
      triangleIndices, "contact candidate triangles copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchPointTriangleContactCandidateMaskKernel(
          devicePositions.data(),
          devicePoints.data(),
          deviceTriangles.data(),
          std::max(0.0, activationDistance),
          deviceSquaredDistances.data(),
          deviceAccepted.data(),
          deviceAcceptedPointIndices.data(),
          deviceAcceptedTriangleIndices.data(),
          deviceAcceptedSquaredDistances.data(),
          deviceCompactedCount.data(),
          deviceAcceptedBlockCounts.data(),
          deviceAcceptedBlockOffsets.data(),
          result.pointCount,
          result.triangleCount),
      "point-triangle contact candidate mask kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(),
      "point-triangle contact candidate mask synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceSquaredDistances.copyFromDevice(
      result.squaredDistances,
      "point-triangle candidate mask squared distances copy");
  deviceAccepted.copyFromDevice(
      result.accepted, "point-triangle candidate mask accepted copy");
  std::vector<std::uint32_t> compactedCount(1u);
  deviceCompactedCount.copyFromDevice(
      compactedCount, "point-triangle candidate mask compacted count copy");
  result.acceptedCount = compactedCount[0];
  result.acceptedPointIndices.resize(result.acceptedCount);
  result.acceptedTriangleIndices.resize(result.acceptedCount);
  result.acceptedSquaredDistances.resize(result.acceptedCount);
  deviceAcceptedPointIndices.copyFromDevice(
      result.acceptedPointIndices,
      "point-triangle candidate mask compacted points copy");
  deviceAcceptedTriangleIndices.copyFromDevice(
      result.acceptedTriangleIndices,
      "point-triangle candidate mask compacted triangles copy");
  deviceAcceptedSquaredDistances.copyFromDevice(
      result.acceptedSquaredDistances,
      "point-triangle candidate mask compacted distances copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (std::size_t i = 0; i < result.accepted.size(); ++i) {
    if (result.accepted[i] != 0) {
      result.maxAcceptedSquaredDistance = std::max(
          result.maxAcceptedSquaredDistance, result.squaredDistances[i]);
    }
  }
}

//==============================================================================
void filterEdgeEdgeContactStencilsCuda(
    const std::vector<double>& positions,
    const std::vector<EdgeEdgeContactStencil>& stencils,
    double activationDistance,
    EdgeEdgeCandidateFilterResult& result)
{
  const auto setupStart = Clock::now();
  static constexpr std::string_view kOperation
      = "filterEdgeEdgeContactStencilsCuda";
  validateInputs(positions, stencils, kOperation);

  result = EdgeEdgeCandidateFilterResult{};
  result.squaredDistances.resize(stencils.size());
  result.accepted.resize(stencils.size());

  if (stencils.empty()) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> devicePositions(positions.size());
  DeviceBuffer<EdgeEdgeContactStencil> deviceStencils(stencils.size());
  DeviceBuffer<double> deviceSquaredDistances(stencils.size());
  DeviceBuffer<std::uint8_t> deviceAccepted(stencils.size());
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  devicePositions.copyToDevice(positions, "edge-edge positions copy");
  deviceStencils.copyToDevice(stencils, "edge-edge stencils copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchEdgeEdgeContactStencilFilterKernel(
          devicePositions.data(),
          deviceStencils.data(),
          std::max(0.0, activationDistance),
          deviceSquaredDistances.data(),
          deviceAccepted.data(),
          stencils.size()),
      "edge-edge contact candidate filter kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(),
      "edge-edge contact candidate filter synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceSquaredDistances.copyFromDevice(
      result.squaredDistances, "edge-edge squared distances copy");
  deviceAccepted.copyFromDevice(result.accepted, "edge-edge accepted copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (std::size_t i = 0; i < result.accepted.size(); ++i) {
    if (result.accepted[i] != 0) {
      ++result.acceptedCount;
      result.maxAcceptedSquaredDistance = std::max(
          result.maxAcceptedSquaredDistance, result.squaredDistances[i]);
    }
  }
}

//==============================================================================
void buildEdgeEdgeContactCandidateMaskCuda(
    const std::vector<double>& positions,
    const std::vector<std::uint32_t>& edgeIndices,
    double activationDistance,
    EdgeEdgeCandidateBuildResult& result)
{
  const auto setupStart = Clock::now();
  static constexpr std::string_view kOperation
      = "buildEdgeEdgeContactCandidateMaskCuda";
  validateEdgeEdgeCandidateMaskInputs(positions, edgeIndices, kOperation);

  result = EdgeEdgeCandidateBuildResult{};
  result.edgeCount = edgeIndices.size() / 2u;
  result.pairCount = result.edgeCount * result.edgeCount;
  result.squaredDistances.resize(result.pairCount);
  result.accepted.resize(result.pairCount);

  if (result.pairCount == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> devicePositions(positions.size());
  DeviceBuffer<std::uint32_t> deviceEdges(edgeIndices.size());
  DeviceBuffer<double> deviceSquaredDistances(result.pairCount);
  DeviceBuffer<std::uint8_t> deviceAccepted(result.pairCount);
  DeviceBuffer<std::uint32_t> deviceAcceptedEdgeAIndices(result.pairCount);
  DeviceBuffer<std::uint32_t> deviceAcceptedEdgeBIndices(result.pairCount);
  DeviceBuffer<double> deviceAcceptedSquaredDistances(result.pairCount);
  DeviceBuffer<std::uint32_t> deviceCompactedCount(1u);
  const std::size_t compactBlockCount
      = (result.pairCount + kContactCandidateMaskBlockSize - 1u)
        / kContactCandidateMaskBlockSize;
  DeviceBuffer<std::uint32_t> deviceAcceptedBlockCounts(compactBlockCount);
  DeviceBuffer<std::uint32_t> deviceAcceptedBlockOffsets(compactBlockCount);
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  devicePositions.copyToDevice(
      positions, "edge-edge candidate mask positions copy");
  deviceEdges.copyToDevice(edgeIndices, "edge-edge candidate mask edges copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchEdgeEdgeContactCandidateMaskKernel(
          devicePositions.data(),
          deviceEdges.data(),
          std::max(0.0, activationDistance),
          deviceSquaredDistances.data(),
          deviceAccepted.data(),
          deviceAcceptedEdgeAIndices.data(),
          deviceAcceptedEdgeBIndices.data(),
          deviceAcceptedSquaredDistances.data(),
          deviceCompactedCount.data(),
          deviceAcceptedBlockCounts.data(),
          deviceAcceptedBlockOffsets.data(),
          result.edgeCount),
      "edge-edge contact candidate mask kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "edge-edge contact candidate mask synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceSquaredDistances.copyFromDevice(
      result.squaredDistances,
      "edge-edge candidate mask squared distances copy");
  deviceAccepted.copyFromDevice(
      result.accepted, "edge-edge candidate mask accepted copy");
  std::vector<std::uint32_t> compactedCount(1u);
  deviceCompactedCount.copyFromDevice(
      compactedCount, "edge-edge candidate mask compacted count copy");
  result.acceptedCount = compactedCount[0];
  result.acceptedEdgeAIndices.resize(result.acceptedCount);
  result.acceptedEdgeBIndices.resize(result.acceptedCount);
  result.acceptedSquaredDistances.resize(result.acceptedCount);
  deviceAcceptedEdgeAIndices.copyFromDevice(
      result.acceptedEdgeAIndices,
      "edge-edge candidate mask compacted edge-a copy");
  deviceAcceptedEdgeBIndices.copyFromDevice(
      result.acceptedEdgeBIndices,
      "edge-edge candidate mask compacted edge-b copy");
  deviceAcceptedSquaredDistances.copyFromDevice(
      result.acceptedSquaredDistances,
      "edge-edge candidate mask compacted distances copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (std::size_t i = 0; i < result.accepted.size(); ++i) {
    if (result.accepted[i] != 0) {
      result.maxAcceptedSquaredDistance = std::max(
          result.maxAcceptedSquaredDistance, result.squaredDistances[i]);
    }
  }
}

//==============================================================================
void buildSweptPointTriangleContactCandidateMaskCuda(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    const std::vector<std::uint32_t>& pointIndices,
    const std::vector<std::uint32_t>& triangleIndices,
    double activationDistance,
    SweptPointTriangleCandidateBuildResult& result)
{
  const auto setupStart = Clock::now();
  static constexpr std::string_view kOperation
      = "buildSweptPointTriangleContactCandidateMaskCuda";
  validateCandidateMaskInputs(
      startPositions, pointIndices, triangleIndices, kOperation);
  validateMatchingEndPositions(startPositions, endPositions, kOperation);

  result = SweptPointTriangleCandidateBuildResult{};
  result.pointCount = pointIndices.size();
  result.triangleCount = triangleIndices.size() / 3u;
  result.pairCount = result.pointCount * result.triangleCount;
  result.endpointSquaredDistances.resize(result.pairCount);
  result.accepted.resize(result.pairCount);

  if (result.pairCount == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> deviceStartPositions(startPositions.size());
  DeviceBuffer<double> deviceEndPositions(endPositions.size());
  DeviceBuffer<std::uint32_t> devicePoints(pointIndices.size());
  DeviceBuffer<std::uint32_t> deviceTriangles(triangleIndices.size());
  DeviceBuffer<double> deviceEndpointSquaredDistances(result.pairCount);
  DeviceBuffer<std::uint8_t> deviceAccepted(result.pairCount);
  DeviceBuffer<std::uint32_t> deviceAcceptedPointIndices(result.pairCount);
  DeviceBuffer<std::uint32_t> deviceAcceptedTriangleIndices(result.pairCount);
  DeviceBuffer<double> deviceAcceptedEndpointSquaredDistances(result.pairCount);
  DeviceBuffer<std::uint32_t> deviceCompactedCount(1u);
  const std::size_t compactBlockCount
      = (result.pairCount + kContactCandidateMaskBlockSize - 1u)
        / kContactCandidateMaskBlockSize;
  DeviceBuffer<std::uint32_t> deviceAcceptedBlockCounts(compactBlockCount);
  DeviceBuffer<std::uint32_t> deviceAcceptedBlockOffsets(compactBlockCount);
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceStartPositions.copyToDevice(
      startPositions, "swept point-triangle start positions copy");
  deviceEndPositions.copyToDevice(
      endPositions, "swept point-triangle end positions copy");
  devicePoints.copyToDevice(
      pointIndices, "swept point-triangle point indices copy");
  deviceTriangles.copyToDevice(
      triangleIndices, "swept point-triangle triangles copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchSweptPointTriangleContactCandidateMaskKernel(
          deviceStartPositions.data(),
          deviceEndPositions.data(),
          devicePoints.data(),
          deviceTriangles.data(),
          std::max(0.0, activationDistance),
          deviceEndpointSquaredDistances.data(),
          deviceAccepted.data(),
          deviceAcceptedPointIndices.data(),
          deviceAcceptedTriangleIndices.data(),
          deviceAcceptedEndpointSquaredDistances.data(),
          deviceCompactedCount.data(),
          deviceAcceptedBlockCounts.data(),
          deviceAcceptedBlockOffsets.data(),
          result.pointCount,
          result.triangleCount),
      "swept point-triangle contact candidate mask kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(),
      "swept point-triangle contact candidate mask synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceEndpointSquaredDistances.copyFromDevice(
      result.endpointSquaredDistances,
      "swept point-triangle endpoint distances copy");
  deviceAccepted.copyFromDevice(
      result.accepted, "swept point-triangle accepted copy");
  std::vector<std::uint32_t> compactedCount(1u);
  deviceCompactedCount.copyFromDevice(
      compactedCount, "swept point-triangle compacted count copy");
  result.acceptedCount = compactedCount[0];
  result.acceptedPointIndices.resize(result.acceptedCount);
  result.acceptedTriangleIndices.resize(result.acceptedCount);
  result.acceptedEndpointSquaredDistances.resize(result.acceptedCount);
  deviceAcceptedPointIndices.copyFromDevice(
      result.acceptedPointIndices,
      "swept point-triangle compacted points copy");
  deviceAcceptedTriangleIndices.copyFromDevice(
      result.acceptedTriangleIndices,
      "swept point-triangle compacted triangles copy");
  deviceAcceptedEndpointSquaredDistances.copyFromDevice(
      result.acceptedEndpointSquaredDistances,
      "swept point-triangle compacted endpoint distances copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (std::size_t i = 0; i < result.accepted.size(); ++i) {
    if (result.accepted[i] != 0) {
      result.maxAcceptedEndpointSquaredDistance = std::max(
          result.maxAcceptedEndpointSquaredDistance,
          result.endpointSquaredDistances[i]);
    }
  }
}

//==============================================================================
void buildSweptEdgeEdgeContactCandidateMaskCuda(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    const std::vector<std::uint32_t>& edgeIndices,
    double activationDistance,
    SweptEdgeEdgeCandidateBuildResult& result)
{
  const auto setupStart = Clock::now();
  static constexpr std::string_view kOperation
      = "buildSweptEdgeEdgeContactCandidateMaskCuda";
  validateEdgeEdgeCandidateMaskInputs(startPositions, edgeIndices, kOperation);
  validateMatchingEndPositions(startPositions, endPositions, kOperation);

  result = SweptEdgeEdgeCandidateBuildResult{};
  result.edgeCount = edgeIndices.size() / 2u;
  result.pairCount = result.edgeCount * result.edgeCount;
  result.endpointSquaredDistances.resize(result.pairCount);
  result.accepted.resize(result.pairCount);

  if (result.pairCount == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> deviceStartPositions(startPositions.size());
  DeviceBuffer<double> deviceEndPositions(endPositions.size());
  DeviceBuffer<std::uint32_t> deviceEdges(edgeIndices.size());
  DeviceBuffer<double> deviceEndpointSquaredDistances(result.pairCount);
  DeviceBuffer<std::uint8_t> deviceAccepted(result.pairCount);
  DeviceBuffer<std::uint32_t> deviceAcceptedEdgeAIndices(result.pairCount);
  DeviceBuffer<std::uint32_t> deviceAcceptedEdgeBIndices(result.pairCount);
  DeviceBuffer<double> deviceAcceptedEndpointSquaredDistances(result.pairCount);
  DeviceBuffer<std::uint32_t> deviceCompactedCount(1u);
  const std::size_t compactBlockCount
      = (result.pairCount + kContactCandidateMaskBlockSize - 1u)
        / kContactCandidateMaskBlockSize;
  DeviceBuffer<std::uint32_t> deviceAcceptedBlockCounts(compactBlockCount);
  DeviceBuffer<std::uint32_t> deviceAcceptedBlockOffsets(compactBlockCount);
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceStartPositions.copyToDevice(
      startPositions, "swept edge-edge start positions copy");
  deviceEndPositions.copyToDevice(
      endPositions, "swept edge-edge end positions copy");
  deviceEdges.copyToDevice(edgeIndices, "swept edge-edge edges copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchSweptEdgeEdgeContactCandidateMaskKernel(
          deviceStartPositions.data(),
          deviceEndPositions.data(),
          deviceEdges.data(),
          std::max(0.0, activationDistance),
          deviceEndpointSquaredDistances.data(),
          deviceAccepted.data(),
          deviceAcceptedEdgeAIndices.data(),
          deviceAcceptedEdgeBIndices.data(),
          deviceAcceptedEndpointSquaredDistances.data(),
          deviceCompactedCount.data(),
          deviceAcceptedBlockCounts.data(),
          deviceAcceptedBlockOffsets.data(),
          result.edgeCount),
      "swept edge-edge contact candidate mask kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "swept edge-edge contact candidate synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceEndpointSquaredDistances.copyFromDevice(
      result.endpointSquaredDistances,
      "swept edge-edge endpoint distances copy");
  deviceAccepted.copyFromDevice(
      result.accepted, "swept edge-edge accepted copy");
  std::vector<std::uint32_t> compactedCount(1u);
  deviceCompactedCount.copyFromDevice(
      compactedCount, "swept edge-edge compacted count copy");
  result.acceptedCount = compactedCount[0];
  result.acceptedEdgeAIndices.resize(result.acceptedCount);
  result.acceptedEdgeBIndices.resize(result.acceptedCount);
  result.acceptedEndpointSquaredDistances.resize(result.acceptedCount);
  deviceAcceptedEdgeAIndices.copyFromDevice(
      result.acceptedEdgeAIndices, "swept edge-edge compacted edge-a copy");
  deviceAcceptedEdgeBIndices.copyFromDevice(
      result.acceptedEdgeBIndices, "swept edge-edge compacted edge-b copy");
  deviceAcceptedEndpointSquaredDistances.copyFromDevice(
      result.acceptedEndpointSquaredDistances,
      "swept edge-edge compacted endpoint distances copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (std::size_t i = 0; i < result.accepted.size(); ++i) {
    if (result.accepted[i] != 0) {
      result.maxAcceptedEndpointSquaredDistance = std::max(
          result.maxAcceptedEndpointSquaredDistance,
          result.endpointSquaredDistances[i]);
    }
  }
}

//==============================================================================
void buildSweptPointTriangleSweepCuda(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    const std::vector<std::uint32_t>& pointIndices,
    const std::vector<std::uint32_t>& triangleIndices,
    double activationDistance,
    SweptPointTriangleSweepResult& result)
{
  const auto setupStart = Clock::now();
  static constexpr std::string_view kOperation
      = "buildSweptPointTriangleSweepCuda";
  validateCandidateMaskInputs(
      startPositions, pointIndices, triangleIndices, kOperation);
  validateMatchingEndPositions(startPositions, endPositions, kOperation);

  result = SweptPointTriangleSweepResult{};
  result.pointCount = pointIndices.size();
  result.triangleCount = triangleIndices.size() / 3u;
  result.pairCapacity = result.pointCount * result.triangleCount;

  if (result.pairCapacity == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  const std::size_t paddedPointCount = nextPowerOfTwo(result.pointCount);
  const std::size_t paddedTriangleCount = nextPowerOfTwo(result.triangleCount);
  DeviceBuffer<double> deviceStartPositions(startPositions.size());
  DeviceBuffer<double> deviceEndPositions(endPositions.size());
  DeviceBuffer<std::uint32_t> devicePoints(pointIndices.size());
  DeviceBuffer<std::uint32_t> deviceTriangles(triangleIndices.size());
  DeviceBuffer<detail::ContactCandidateSweepAabbItem> devicePointItems(
      paddedPointCount);
  DeviceBuffer<detail::ContactCandidateSweepAabbItem> deviceTriangleItems(
      paddedTriangleCount);
  DeviceBuffer<std::uint32_t> devicePairCounts(result.pointCount);
  DeviceBuffer<std::uint32_t> devicePairOffsets(result.pointCount);
  DeviceBuffer<std::uint32_t> deviceCompactedCount(1u);
  DeviceBuffer<std::uint32_t> deviceAcceptedPointIndices(result.pairCapacity);
  DeviceBuffer<std::uint32_t> deviceAcceptedTriangleIndices(
      result.pairCapacity);
  DeviceBuffer<double> deviceAcceptedEndpointSquaredDistances(
      result.pairCapacity);
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceStartPositions.copyToDevice(
      startPositions, "swept point-triangle sweep start positions copy");
  deviceEndPositions.copyToDevice(
      endPositions, "swept point-triangle sweep end positions copy");
  devicePoints.copyToDevice(
      pointIndices, "swept point-triangle sweep point indices copy");
  deviceTriangles.copyToDevice(
      triangleIndices, "swept point-triangle sweep triangles copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchSweptPointTriangleSweepKernel(
          deviceStartPositions.data(),
          deviceEndPositions.data(),
          devicePoints.data(),
          deviceTriangles.data(),
          std::max(0.0, activationDistance),
          devicePointItems.data(),
          deviceTriangleItems.data(),
          devicePairCounts.data(),
          devicePairOffsets.data(),
          deviceCompactedCount.data(),
          deviceAcceptedPointIndices.data(),
          deviceAcceptedTriangleIndices.data(),
          deviceAcceptedEndpointSquaredDistances.data(),
          result.pointCount,
          result.triangleCount,
          paddedPointCount,
          paddedTriangleCount),
      "swept point-triangle sweep kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "swept point-triangle sweep synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  std::vector<std::uint32_t> compactedCount(1u);
  deviceCompactedCount.copyFromDevice(
      compactedCount, "swept point-triangle sweep compacted count copy");
  result.acceptedCount = compactedCount[0];
  result.acceptedPointIndices.resize(result.acceptedCount);
  result.acceptedTriangleIndices.resize(result.acceptedCount);
  result.acceptedEndpointSquaredDistances.resize(result.acceptedCount);
  deviceAcceptedPointIndices.copyFromDevice(
      result.acceptedPointIndices,
      "swept point-triangle sweep compacted points copy");
  deviceAcceptedTriangleIndices.copyFromDevice(
      result.acceptedTriangleIndices,
      "swept point-triangle sweep compacted triangles copy");
  deviceAcceptedEndpointSquaredDistances.copyFromDevice(
      result.acceptedEndpointSquaredDistances,
      "swept point-triangle sweep compacted endpoint distances copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (const double distance : result.acceptedEndpointSquaredDistances) {
    result.maxAcceptedEndpointSquaredDistance
        = std::max(result.maxAcceptedEndpointSquaredDistance, distance);
  }
}

//==============================================================================
void buildSweptEdgeEdgeSweepCuda(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    const std::vector<std::uint32_t>& edgeIndices,
    double activationDistance,
    SweptEdgeEdgeSweepResult& result)
{
  const auto setupStart = Clock::now();
  static constexpr std::string_view kOperation = "buildSweptEdgeEdgeSweepCuda";
  validateEdgeEdgeCandidateMaskInputs(startPositions, edgeIndices, kOperation);
  validateMatchingEndPositions(startPositions, endPositions, kOperation);

  result = SweptEdgeEdgeSweepResult{};
  result.edgeCount = edgeIndices.size() / 2u;
  result.pairCapacity = result.edgeCount * result.edgeCount;

  if (result.pairCapacity == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  const std::size_t paddedEdgeCount = nextPowerOfTwo(result.edgeCount);
  DeviceBuffer<double> deviceStartPositions(startPositions.size());
  DeviceBuffer<double> deviceEndPositions(endPositions.size());
  DeviceBuffer<std::uint32_t> deviceEdges(edgeIndices.size());
  DeviceBuffer<detail::ContactCandidateSweepAabbItem> deviceEdgeItems(
      paddedEdgeCount);
  DeviceBuffer<std::uint32_t> devicePairCounts(result.edgeCount);
  DeviceBuffer<std::uint32_t> devicePairOffsets(result.edgeCount);
  DeviceBuffer<std::uint32_t> deviceCompactedCount(1u);
  DeviceBuffer<std::uint32_t> deviceAcceptedEdgeAIndices(result.pairCapacity);
  DeviceBuffer<std::uint32_t> deviceAcceptedEdgeBIndices(result.pairCapacity);
  DeviceBuffer<double> deviceAcceptedEndpointSquaredDistances(
      result.pairCapacity);
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceStartPositions.copyToDevice(
      startPositions, "swept edge-edge sweep start positions copy");
  deviceEndPositions.copyToDevice(
      endPositions, "swept edge-edge sweep end positions copy");
  deviceEdges.copyToDevice(edgeIndices, "swept edge-edge sweep edges copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchSweptEdgeEdgeSweepKernel(
          deviceStartPositions.data(),
          deviceEndPositions.data(),
          deviceEdges.data(),
          std::max(0.0, activationDistance),
          deviceEdgeItems.data(),
          devicePairCounts.data(),
          devicePairOffsets.data(),
          deviceCompactedCount.data(),
          deviceAcceptedEdgeAIndices.data(),
          deviceAcceptedEdgeBIndices.data(),
          deviceAcceptedEndpointSquaredDistances.data(),
          result.edgeCount,
          paddedEdgeCount),
      "swept edge-edge sweep kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "swept edge-edge sweep synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  std::vector<std::uint32_t> compactedCount(1u);
  deviceCompactedCount.copyFromDevice(
      compactedCount, "swept edge-edge sweep compacted count copy");
  result.acceptedCount = compactedCount[0];
  result.acceptedEdgeAIndices.resize(result.acceptedCount);
  result.acceptedEdgeBIndices.resize(result.acceptedCount);
  result.acceptedEndpointSquaredDistances.resize(result.acceptedCount);
  deviceAcceptedEdgeAIndices.copyFromDevice(
      result.acceptedEdgeAIndices, "swept edge-edge sweep edge-a copy");
  deviceAcceptedEdgeBIndices.copyFromDevice(
      result.acceptedEdgeBIndices, "swept edge-edge sweep edge-b copy");
  deviceAcceptedEndpointSquaredDistances.copyFromDevice(
      result.acceptedEndpointSquaredDistances,
      "swept edge-edge sweep compacted endpoint distances copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (const double distance : result.acceptedEndpointSquaredDistances) {
    result.maxAcceptedEndpointSquaredDistance
        = std::max(result.maxAcceptedEndpointSquaredDistance, distance);
  }
}

//==============================================================================
void evaluateSweptPointTriangleCandidateBufferCuda(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    const std::vector<std::uint32_t>& triangleIndices,
    const std::vector<std::uint32_t>& candidatePointIndices,
    const std::vector<std::uint32_t>& candidateTriangleIndices,
    SweptPointTriangleCandidateBufferResult& result)
{
  const auto setupStart = Clock::now();
  static constexpr std::string_view kOperation
      = "evaluateSweptPointTriangleCandidateBufferCuda";
  validatePointTriangleCandidateBufferInputs(
      startPositions,
      triangleIndices,
      candidatePointIndices,
      candidateTriangleIndices,
      kOperation);
  validateMatchingEndPositions(startPositions, endPositions, kOperation);

  result = SweptPointTriangleCandidateBufferResult{};
  result.pointCount = startPositions.size() / 3u;
  result.triangleCount = triangleIndices.size() / 3u;
  result.pairCount = candidatePointIndices.size();
  result.endpointSquaredDistances.resize(result.pairCount);

  if (result.pairCount == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> deviceStartPositions(startPositions.size());
  DeviceBuffer<double> deviceEndPositions(endPositions.size());
  DeviceBuffer<std::uint32_t> deviceTriangles(triangleIndices.size());
  DeviceBuffer<std::uint32_t> deviceCandidatePoints(
      candidatePointIndices.size());
  DeviceBuffer<std::uint32_t> deviceCandidateTriangles(
      candidateTriangleIndices.size());
  DeviceBuffer<double> deviceEndpointSquaredDistances(result.pairCount);
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceStartPositions.copyToDevice(
      startPositions, "runtime point-triangle start positions copy");
  deviceEndPositions.copyToDevice(
      endPositions, "runtime point-triangle end positions copy");
  deviceTriangles.copyToDevice(
      triangleIndices, "runtime point-triangle triangles copy");
  deviceCandidatePoints.copyToDevice(
      candidatePointIndices, "runtime point-triangle candidate points copy");
  deviceCandidateTriangles.copyToDevice(
      candidateTriangleIndices,
      "runtime point-triangle candidate triangles copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchSweptPointTriangleCandidateBufferKernel(
          deviceStartPositions.data(),
          deviceEndPositions.data(),
          deviceTriangles.data(),
          deviceCandidatePoints.data(),
          deviceCandidateTriangles.data(),
          deviceEndpointSquaredDistances.data(),
          result.pairCount),
      "runtime point-triangle candidate buffer kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(),
      "runtime point-triangle candidate buffer synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceEndpointSquaredDistances.copyFromDevice(
      result.endpointSquaredDistances,
      "runtime point-triangle endpoint distances copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (const double distance : result.endpointSquaredDistances) {
    result.maxEndpointSquaredDistance
        = std::max(result.maxEndpointSquaredDistance, distance);
  }
}

//==============================================================================
void evaluateSweptEdgeEdgeCandidateBufferCuda(
    const std::vector<double>& startPositions,
    const std::vector<double>& endPositions,
    const std::vector<std::uint32_t>& edgeIndices,
    const std::vector<std::uint32_t>& candidateEdgeAIndices,
    const std::vector<std::uint32_t>& candidateEdgeBIndices,
    SweptEdgeEdgeCandidateBufferResult& result)
{
  const auto setupStart = Clock::now();
  static constexpr std::string_view kOperation
      = "evaluateSweptEdgeEdgeCandidateBufferCuda";
  validateEdgeEdgeCandidateBufferInputs(
      startPositions,
      edgeIndices,
      candidateEdgeAIndices,
      candidateEdgeBIndices,
      kOperation);
  validateMatchingEndPositions(startPositions, endPositions, kOperation);

  result = SweptEdgeEdgeCandidateBufferResult{};
  result.edgeCount = edgeIndices.size() / 2u;
  result.pairCount = candidateEdgeAIndices.size();
  result.endpointSquaredDistances.resize(result.pairCount);

  if (result.pairCount == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> deviceStartPositions(startPositions.size());
  DeviceBuffer<double> deviceEndPositions(endPositions.size());
  DeviceBuffer<std::uint32_t> deviceEdges(edgeIndices.size());
  DeviceBuffer<std::uint32_t> deviceCandidateEdgeA(
      candidateEdgeAIndices.size());
  DeviceBuffer<std::uint32_t> deviceCandidateEdgeB(
      candidateEdgeBIndices.size());
  DeviceBuffer<double> deviceEndpointSquaredDistances(result.pairCount);
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceStartPositions.copyToDevice(
      startPositions, "runtime edge-edge start positions copy");
  deviceEndPositions.copyToDevice(
      endPositions, "runtime edge-edge end positions copy");
  deviceEdges.copyToDevice(edgeIndices, "runtime edge-edge edges copy");
  deviceCandidateEdgeA.copyToDevice(
      candidateEdgeAIndices, "runtime edge-edge candidate edge-a copy");
  deviceCandidateEdgeB.copyToDevice(
      candidateEdgeBIndices, "runtime edge-edge candidate edge-b copy");
  const auto h2dEnd = Clock::now();

  const auto kernelStart = Clock::now();
  throwIfCudaError(
      detail::launchSweptEdgeEdgeCandidateBufferKernel(
          deviceStartPositions.data(),
          deviceEndPositions.data(),
          deviceEdges.data(),
          deviceCandidateEdgeA.data(),
          deviceCandidateEdgeB.data(),
          deviceEndpointSquaredDistances.data(),
          result.pairCount),
      "runtime edge-edge candidate buffer kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(),
      "runtime edge-edge candidate buffer synchronize");
  const auto kernelEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceEndpointSquaredDistances.copyFromDevice(
      result.endpointSquaredDistances,
      "runtime edge-edge endpoint distances copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.kernelNs = elapsedNs(kernelStart, kernelEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (const double distance : result.endpointSquaredDistances) {
    result.maxEndpointSquaredDistance
        = std::max(result.maxEndpointSquaredDistance, distance);
  }
}

} // namespace dart::simulation::compute::cuda
