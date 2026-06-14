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
 *     copyright notice, this list of conditions and the disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
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
#include <dart/simulation/compute/cuda/newton_assembly_solve_cuda.cuh>

#include <algorithm>
#include <chrono>

#include <cmath>

namespace sx = dart::simulation;

namespace dart::simulation::compute::cuda {
namespace detail {

cudaError_t launchNewtonAssemblyRowsKernel(
    const NewtonAssemblySolveRowInput* rows,
    double* assembledDiagonal,
    double* assembledGradient,
    std::size_t rowCount);

cudaError_t launchNewtonOffDiagonalAssemblyRowsKernel(
    const NewtonOffDiagonalAssemblyRowInput* rows,
    double* assembledBlocks,
    std::size_t rowCount);

cudaError_t launchNewtonDiagonalSolveKernel(
    const double* assembledDiagonal,
    const double* assembledGradient,
    double* step,
    double* residual,
    std::size_t dofCount,
    double regularization,
    double epsilonForDivision);

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

void validateRows(
    const std::vector<NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const double regularization)
{
  DART_SIMULATION_THROW_T_IF(
      !std::isfinite(regularization) || regularization < 0.0,
      sx::InvalidArgumentException,
      "evaluateNewtonAssemblySolveCuda regularization must be non-negative");
  DART_SIMULATION_THROW_T_IF(
      bodyCount == 0 && !rows.empty(),
      sx::InvalidArgumentException,
      "evaluateNewtonAssemblySolveCuda requires at least one body for rows");

  for (std::size_t row = 0; row < rows.size(); ++row) {
    const auto& input = rows[row];
    DART_SIMULATION_THROW_T_IF(
        input.bodyIndex >= bodyCount,
        sx::InvalidArgumentException,
        "evaluateNewtonAssemblySolveCuda row {} body index {} is outside {} "
        "bodies",
        row,
        input.bodyIndex,
        bodyCount);
    for (std::size_t dof = 0; dof < kNewtonAssemblySolveDofsPerBody; ++dof) {
      DART_SIMULATION_THROW_T_IF(
          !std::isfinite(input.hessianDiagonal[dof])
              || input.hessianDiagonal[dof] < 0.0
              || !std::isfinite(input.gradient[dof]),
          sx::InvalidArgumentException,
          "evaluateNewtonAssemblySolveCuda row {} has an invalid dof {}",
          row,
          dof);
    }
  }
}

void validateOffDiagonalRows(
    const std::vector<NewtonOffDiagonalAssemblyRowInput>& rows,
    const std::size_t pairCount)
{
  DART_SIMULATION_THROW_T_IF(
      pairCount == 0 && !rows.empty(),
      sx::InvalidArgumentException,
      "evaluateNewtonOffDiagonalAssemblyCuda requires at least one pair for "
      "rows");

  for (std::size_t row = 0; row < rows.size(); ++row) {
    const auto& input = rows[row];
    DART_SIMULATION_THROW_T_IF(
        input.pairIndex >= pairCount,
        sx::InvalidArgumentException,
        "evaluateNewtonOffDiagonalAssemblyCuda row {} pair index {} is outside "
        "{} pairs",
        row,
        input.pairIndex,
        pairCount);
    for (std::size_t entry = 0; entry < kNewtonAssemblySolveBlockEntries;
         ++entry) {
      DART_SIMULATION_THROW_T_IF(
          !std::isfinite(input.hessianBlock[entry]),
          sx::InvalidArgumentException,
          "evaluateNewtonOffDiagonalAssemblyCuda row {} has an invalid block "
          "entry {}",
          row,
          entry);
    }
  }
}

} // namespace

//==============================================================================
void evaluateNewtonAssemblySolveCuda(
    const std::vector<NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const double regularization,
    NewtonAssemblySolveResult& result)
{
  const auto setupStart = Clock::now();
  validateRows(rows, bodyCount, regularization);

  result = NewtonAssemblySolveResult{};
  result.bodyCount = bodyCount;
  result.rowCount = rows.size();
  const std::size_t dofCount = bodyCount * kNewtonAssemblySolveDofsPerBody;
  result.assembledDiagonal.assign(dofCount, 0.0);
  result.assembledGradient.assign(dofCount, 0.0);
  result.step.assign(dofCount, 0.0);
  result.residual.assign(dofCount, 0.0);

  if (dofCount == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<NewtonAssemblySolveRowInput> deviceRows(rows.size());
  DeviceBuffer<double> deviceDiagonal(dofCount);
  DeviceBuffer<double> deviceGradient(dofCount);
  DeviceBuffer<double> deviceStep(dofCount);
  DeviceBuffer<double> deviceResidual(dofCount);
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceRows.copyToDevice(rows, "newton assembly rows copy");
  throwIfCudaError(
      cudaMemset(deviceDiagonal.data(), 0, deviceDiagonal.byteSize()),
      "newton assembly diagonal zero");
  throwIfCudaError(
      cudaMemset(deviceGradient.data(), 0, deviceGradient.byteSize()),
      "newton assembly gradient zero");
  const auto h2dEnd = Clock::now();

  const auto assemblyStart = Clock::now();
  throwIfCudaError(
      detail::launchNewtonAssemblyRowsKernel(
          deviceRows.data(),
          deviceDiagonal.data(),
          deviceGradient.data(),
          rows.size()),
      "newton assembly rows kernel");
  throwIfCudaError(cudaDeviceSynchronize(), "newton assembly rows synchronize");
  const auto assemblyEnd = Clock::now();

  const auto solveStart = Clock::now();
  throwIfCudaError(
      detail::launchNewtonDiagonalSolveKernel(
          deviceDiagonal.data(),
          deviceGradient.data(),
          deviceStep.data(),
          deviceResidual.data(),
          dofCount,
          regularization,
          1e-14),
      "newton diagonal solve kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "newton diagonal solve synchronize");
  const auto solveEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceDiagonal.copyFromDevice(
      result.assembledDiagonal, "newton assembled diagonal copy");
  deviceGradient.copyFromDevice(
      result.assembledGradient, "newton assembled gradient copy");
  deviceStep.copyFromDevice(result.step, "newton step copy");
  deviceResidual.copyFromDevice(result.residual, "newton residual copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.assemblyKernelNs = elapsedNs(assemblyStart, assemblyEnd);
  result.timing.solveKernelNs = elapsedNs(solveStart, solveEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  double stepNormSquared = 0.0;
  double residualNormSquared = 0.0;
  for (std::size_t dof = 0; dof < dofCount; ++dof) {
    const double effectiveDiagonal
        = result.assembledDiagonal[dof] + regularization;
    if (effectiveDiagonal > 1e-14) {
      ++result.activeDofCount;
    }
    result.maxDiagonal
        = std::max(result.maxDiagonal, result.assembledDiagonal[dof]);
    result.maxGradientAbs = std::max(
        result.maxGradientAbs, std::abs(result.assembledGradient[dof]));
    stepNormSquared += result.step[dof] * result.step[dof];
    residualNormSquared += result.residual[dof] * result.residual[dof];
  }
  result.stepNorm = std::sqrt(stepNormSquared);
  result.residualNorm = std::sqrt(residualNormSquared);
}

//==============================================================================
void evaluateNewtonOffDiagonalAssemblyCuda(
    const std::vector<NewtonOffDiagonalAssemblyRowInput>& rows,
    const std::size_t pairCount,
    NewtonOffDiagonalAssemblyResult& result)
{
  const auto setupStart = Clock::now();
  validateOffDiagonalRows(rows, pairCount);

  result = NewtonOffDiagonalAssemblyResult{};
  result.pairCount = pairCount;
  result.rowCount = rows.size();
  const std::size_t entryCount = pairCount * kNewtonAssemblySolveBlockEntries;
  result.assembledBlocks.assign(entryCount, 0.0);

  if (entryCount == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<NewtonOffDiagonalAssemblyRowInput> deviceRows(rows.size());
  DeviceBuffer<double> deviceBlocks(entryCount);
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceRows.copyToDevice(rows, "newton off-diagonal assembly rows copy");
  throwIfCudaError(
      cudaMemset(deviceBlocks.data(), 0, deviceBlocks.byteSize()),
      "newton off-diagonal assembly blocks zero");
  const auto h2dEnd = Clock::now();

  const auto assemblyStart = Clock::now();
  throwIfCudaError(
      detail::launchNewtonOffDiagonalAssemblyRowsKernel(
          deviceRows.data(), deviceBlocks.data(), rows.size()),
      "newton off-diagonal assembly rows kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "newton off-diagonal assembly rows synchronize");
  const auto assemblyEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceBlocks.copyFromDevice(
      result.assembledBlocks, "newton off-diagonal assembled blocks copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.assemblyKernelNs = elapsedNs(assemblyStart, assemblyEnd);
  result.timing.solveKernelNs = 0.0;
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  for (std::size_t pair = 0; pair < pairCount; ++pair) {
    bool active = false;
    const std::size_t offset = pair * kNewtonAssemblySolveBlockEntries;
    for (std::size_t entry = 0; entry < kNewtonAssemblySolveBlockEntries;
         ++entry) {
      const double value = std::abs(result.assembledBlocks[offset + entry]);
      result.maxBlockAbs = std::max(result.maxBlockAbs, value);
      active = active || value > 0.0;
    }
    if (active) {
      ++result.activeBlockCount;
    }
  }
}

} // namespace dart::simulation::compute::cuda
