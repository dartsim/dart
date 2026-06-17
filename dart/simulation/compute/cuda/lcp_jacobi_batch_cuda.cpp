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

#include <dart/simulation/common/exceptions.hpp>

#include <cuda_runtime_api.h>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/device_buffer.cuh>
#include <dart/simulation/compute/cuda/lcp_jacobi_batch_cuda.cuh>

#include <algorithm>
#include <limits>
#include <string_view>

#include <cmath>
#include <cstddef>

namespace sx = dart::simulation;

namespace dart::simulation::compute::cuda {
namespace detail {

cudaError_t launchBoxedLcpJacobiBatchKernel(
    const double* A,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex,
    const double* x,
    double* xNext,
    std::size_t problemSize,
    std::size_t problemCount,
    double relaxation,
    double epsilonForDivision);

cudaError_t launchBoxedLcpPgsBatchKernel(
    const double* A,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex,
    double* x,
    std::size_t problemSize,
    std::size_t problemCount,
    std::size_t iterations,
    double relaxation,
    double epsilonForDivision);

cudaError_t launchBoxedLcpRedBlackGaussSeidelBatchKernel(
    const double* A,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex,
    double* x,
    const double* xPrevious,
    std::size_t problemSize,
    std::size_t problemCount,
    int color,
    double relaxation,
    double epsilonForDivision);

} // namespace detail
namespace {

std::size_t checkedProduct(
    std::size_t a, std::size_t b, std::string_view operation)
{
  DART_SIMULATION_THROW_T_IF(
      a != 0 && b > std::numeric_limits<std::size_t>::max() / a,
      sx::InvalidArgumentException,
      "{} size overflow",
      operation);
  return a * b;
}

bool vectorIsFinite(const std::vector<double>& values)
{
  return std::ranges::all_of(
      values, [](double value) { return std::isfinite(value); });
}

bool boundsAreValid(
    const std::vector<double>& lo, const std::vector<double>& hi)
{
  for (std::size_t i = 0; i < lo.size(); ++i) {
    if (std::isnan(lo[i]) || std::isnan(hi[i]) || lo[i] > hi[i]) {
      return false;
    }
  }
  return true;
}

bool findexIsValid(
    const std::vector<int>& findex,
    std::size_t problemSize,
    std::size_t problemCount)
{
  for (std::size_t problem = 0; problem < problemCount; ++problem) {
    const std::size_t base = problem * problemSize;
    for (std::size_t row = 0; row < problemSize; ++row) {
      const int ref = findex[base + row];
      if (ref < 0) {
        continue;
      }
      if (static_cast<std::size_t>(ref) >= problemSize
          || static_cast<std::size_t>(ref) == row) {
        return false;
      }
    }
  }
  return true;
}

bool finiteFrictionCoefficients(
    const std::vector<double>& hi, const std::vector<int>& findex)
{
  for (std::size_t i = 0; i < findex.size(); ++i) {
    if (findex[i] >= 0 && !std::isfinite(hi[i])) {
      return false;
    }
  }
  return true;
}

void validateProblem(
    const LcpBatchCudaProblem& problem, std::string_view operation)
{
  DART_SIMULATION_THROW_T_IF(
      problem.iterations == 0,
      sx::InvalidArgumentException,
      "{} requires at least one iteration",
      operation);
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(problem.relaxation) || problem.relaxation <= 0.0
          || problem.relaxation > 2.0,
      sx::InvalidArgumentException,
      "{} relaxation must be in (0, 2]",
      operation);
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(problem.epsilonForDivision)
          || problem.epsilonForDivision <= 0.0,
      sx::InvalidArgumentException,
      "{} division epsilon must be positive",
      operation);

  const std::size_t vectorSize
      = checkedProduct(problem.problemSize, problem.problemCount, operation);
  const std::size_t matrixSize
      = checkedProduct(vectorSize, problem.problemSize, operation);

  DART_SIMULATION_THROW_T_IF(
      problem.A.size() != matrixSize || problem.b.size() != vectorSize
          || problem.lo.size() != vectorSize || problem.hi.size() != vectorSize
          || problem.findex.size() != vectorSize
          || problem.x.size() != vectorSize,
      sx::InvalidArgumentException,
      "{} buffers do not match problemSize {} and problemCount {}",
      operation,
      problem.problemSize,
      problem.problemCount);

  DART_SIMULATION_THROW_T_IF(
      !vectorIsFinite(problem.A) || !vectorIsFinite(problem.b)
          || !vectorIsFinite(problem.x),
      sx::InvalidArgumentException,
      "{} A, b, and x buffers must be finite",
      operation);
  DART_SIMULATION_THROW_T_IF(
      !boundsAreValid(problem.lo, problem.hi),
      sx::InvalidArgumentException,
      "{} bounds must be ordered and non-NaN",
      operation);
  DART_SIMULATION_THROW_T_IF(
      !findexIsValid(problem.findex, problem.problemSize, problem.problemCount),
      sx::InvalidArgumentException,
      "{} findex entries must be local row indices, in range, and not "
      "self-referential",
      operation);
  DART_SIMULATION_THROW_T_IF(
      !finiteFrictionCoefficients(problem.hi, problem.findex),
      sx::InvalidArgumentException,
      "{} friction-index rows require finite hi coefficients",
      operation);
}

void throwIfCudaRuntimeUnavailable()
{
  DART_SIMULATION_THROW_T_IF(
      !isCudaRuntimeAvailable(),
      sx::InvalidOperationException,
      "CUDA runtime has no available device");
}

} // namespace

//==============================================================================
void solveBoxedLcpJacobiBatchCuda(LcpBatchCudaProblem& problem)
{
  validateProblem(problem, "solveBoxedLcpJacobiBatchCuda");

  const std::size_t vectorSize = problem.problemSize * problem.problemCount;
  if (vectorSize == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> deviceA(problem.A);
  DeviceBuffer<double> deviceB(problem.b);
  DeviceBuffer<double> deviceLo(problem.lo);
  DeviceBuffer<double> deviceHi(problem.hi);
  DeviceBuffer<int> deviceFindex(problem.findex);
  DeviceBuffer<double> deviceX(problem.x);
  DeviceBuffer<double> deviceXNext(vectorSize);

  double* current = deviceX.data();
  double* next = deviceXNext.data();
  for (std::size_t iter = 0; iter < problem.iterations; ++iter) {
    throwIfCudaError(
        detail::launchBoxedLcpJacobiBatchKernel(
            deviceA.data(),
            deviceB.data(),
            deviceLo.data(),
            deviceHi.data(),
            deviceFindex.data(),
            current,
            next,
            problem.problemSize,
            problem.problemCount,
            problem.relaxation,
            problem.epsilonForDivision),
        "boxed LCP Jacobi batch kernel");
    std::swap(current, next);
  }

  throwIfCudaError(cudaDeviceSynchronize(), "boxed LCP Jacobi synchronize");

  if (current == deviceX.data()) {
    deviceX.copyFromDevice(problem.x, "LCP solution copy from device");
  } else {
    deviceXNext.copyFromDevice(problem.x, "LCP solution copy from device");
  }
}

//==============================================================================
void solveBoxedLcpRedBlackGaussSeidelBatchCuda(LcpBatchCudaProblem& problem)
{
  validateProblem(problem, "solveBoxedLcpRedBlackGaussSeidelBatchCuda");

  const std::size_t vectorSize = problem.problemSize * problem.problemCount;
  if (vectorSize == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> deviceA(problem.A);
  DeviceBuffer<double> deviceB(problem.b);
  DeviceBuffer<double> deviceLo(problem.lo);
  DeviceBuffer<double> deviceHi(problem.hi);
  DeviceBuffer<int> deviceFindex(problem.findex);
  DeviceBuffer<double> deviceX(problem.x);
  DeviceBuffer<double> deviceXPrevious(vectorSize);

  const std::size_t bytes = vectorSize * sizeof(double);
  for (std::size_t iter = 0; iter < problem.iterations; ++iter) {
    throwIfCudaError(
        cudaMemcpy(
            deviceXPrevious.data(),
            deviceX.data(),
            bytes,
            cudaMemcpyDeviceToDevice),
        "boxed LCP red-black Gauss-Seidel previous iterate copy");
    throwIfCudaError(
        detail::launchBoxedLcpRedBlackGaussSeidelBatchKernel(
            deviceA.data(),
            deviceB.data(),
            deviceLo.data(),
            deviceHi.data(),
            deviceFindex.data(),
            deviceX.data(),
            deviceXPrevious.data(),
            problem.problemSize,
            problem.problemCount,
            0,
            problem.relaxation,
            problem.epsilonForDivision),
        "boxed LCP red-black Gauss-Seidel red kernel");
    throwIfCudaError(
        detail::launchBoxedLcpRedBlackGaussSeidelBatchKernel(
            deviceA.data(),
            deviceB.data(),
            deviceLo.data(),
            deviceHi.data(),
            deviceFindex.data(),
            deviceX.data(),
            deviceXPrevious.data(),
            problem.problemSize,
            problem.problemCount,
            1,
            problem.relaxation,
            problem.epsilonForDivision),
        "boxed LCP red-black Gauss-Seidel black kernel");
  }

  throwIfCudaError(
      cudaDeviceSynchronize(), "boxed LCP red-black Gauss-Seidel synchronize");
  deviceX.copyFromDevice(problem.x, "LCP solution copy from device");
}

//==============================================================================
void solveBoxedLcpPgsBatchCuda(LcpBatchCudaProblem& problem)
{
  validateProblem(problem, "solveBoxedLcpPgsBatchCuda");

  const std::size_t vectorSize = problem.problemSize * problem.problemCount;
  if (vectorSize == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<double> deviceA(problem.A);
  DeviceBuffer<double> deviceB(problem.b);
  DeviceBuffer<double> deviceLo(problem.lo);
  DeviceBuffer<double> deviceHi(problem.hi);
  DeviceBuffer<int> deviceFindex(problem.findex);
  DeviceBuffer<double> deviceX(problem.x);

  throwIfCudaError(
      detail::launchBoxedLcpPgsBatchKernel(
          deviceA.data(),
          deviceB.data(),
          deviceLo.data(),
          deviceHi.data(),
          deviceFindex.data(),
          deviceX.data(),
          problem.problemSize,
          problem.problemCount,
          problem.iterations,
          problem.relaxation,
          problem.epsilonForDivision),
      "boxed LCP PGS batch kernel");

  throwIfCudaError(cudaDeviceSynchronize(), "boxed LCP PGS synchronize");
  deviceX.copyFromDevice(problem.x, "LCP solution copy from device");
}

//==============================================================================
void solveBoxedLcpJacobiGroupedBatchCuda(
    std::vector<LcpBatchCudaProblem>& problemGroups)
{
  for (auto& problem : problemGroups) {
    solveBoxedLcpJacobiBatchCuda(problem);
  }
}

//==============================================================================
void solveBoxedLcpRedBlackGaussSeidelGroupedBatchCuda(
    std::vector<LcpBatchCudaProblem>& problemGroups)
{
  for (auto& problem : problemGroups) {
    solveBoxedLcpRedBlackGaussSeidelBatchCuda(problem);
  }
}

//==============================================================================
void solveBoxedLcpPgsGroupedBatchCuda(
    std::vector<LcpBatchCudaProblem>& problemGroups)
{
  for (auto& problem : problemGroups) {
    solveBoxedLcpPgsBatchCuda(problem);
  }
}

} // namespace dart::simulation::compute::cuda
