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

} // namespace dart::simulation::compute::cuda
