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

} // namespace detail
namespace {

using Clock = std::chrono::steady_clock;

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

} // namespace dart::simulation::compute::cuda
