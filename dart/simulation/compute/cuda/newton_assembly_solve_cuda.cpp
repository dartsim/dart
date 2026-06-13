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

cudaError_t launchNewtonEqualityReductionKernel(
    const NewtonEqualityReductionEntry* entries,
    const double* assembledDiagonal,
    const double* assembledGradient,
    double* reducedDiagonal,
    double* reducedGradient,
    std::size_t entryCount);

cudaError_t launchNewtonSparseDiagonalResidualKernel(
    const double* assembledDiagonal,
    const double* step,
    double* residual,
    std::size_t dofCount,
    double regularization);

cudaError_t launchNewtonSparseBlockResidualKernel(
    const NewtonSparseBlockEntry* blocks,
    const double* step,
    double* residual,
    std::size_t blockCount);

cudaError_t launchNewtonDiagonalSolveKernel(
    const double* assembledDiagonal,
    const double* assembledGradient,
    double* step,
    double* residual,
    std::size_t dofCount,
    double regularization,
    double epsilonForDivision);

cudaError_t launchNewtonExpandEqualityReducedStepKernel(
    const NewtonEqualityReductionEntry* entries,
    const double* reducedStep,
    double* fullStep,
    std::size_t entryCount);

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

void validateSparseBlocks(
    const std::vector<NewtonSparseBlockEntry>& blocks,
    const std::size_t bodyCount)
{
  DART_SIMULATION_THROW_T_IF(
      bodyCount == 0 && !blocks.empty(),
      sx::InvalidArgumentException,
      "evaluateNewtonSparseResidualCuda requires bodies for sparse blocks");

  for (std::size_t block = 0; block < blocks.size(); ++block) {
    const auto& input = blocks[block];
    DART_SIMULATION_THROW_T_IF(
        input.rowBodyIndex >= bodyCount,
        sx::InvalidArgumentException,
        "evaluateNewtonSparseResidualCuda block {} row body {} is outside {} "
        "bodies",
        block,
        input.rowBodyIndex,
        bodyCount);
    DART_SIMULATION_THROW_T_IF(
        input.columnBodyIndex >= bodyCount,
        sx::InvalidArgumentException,
        "evaluateNewtonSparseResidualCuda block {} column body {} is outside "
        "{} bodies",
        block,
        input.columnBodyIndex,
        bodyCount);
    DART_SIMULATION_THROW_T_IF(
        input.rowBodyIndex == input.columnBodyIndex,
        sx::InvalidArgumentException,
        "evaluateNewtonSparseResidualCuda block {} must connect different "
        "bodies",
        block);
    for (std::size_t entry = 0; entry < kNewtonAssemblySolveBlockEntries;
         ++entry) {
      DART_SIMULATION_THROW_T_IF(
          !std::isfinite(input.hessianBlock[entry]),
          sx::InvalidArgumentException,
          "evaluateNewtonSparseResidualCuda block {} has an invalid entry {}",
          block,
          entry);
    }
  }
}

void validateStep(
    const std::vector<double>& step, const std::size_t expectedDofCount)
{
  DART_SIMULATION_THROW_T_IF(
      step.size() != expectedDofCount,
      sx::InvalidArgumentException,
      "evaluateNewtonSparseResidualCuda requires {} step dofs, got {}",
      expectedDofCount,
      step.size());
  for (std::size_t dof = 0; dof < step.size(); ++dof) {
    DART_SIMULATION_THROW_T_IF(
        !std::isfinite(step[dof]),
        sx::InvalidArgumentException,
        "evaluateNewtonSparseResidualCuda step dof {} is invalid",
        dof);
  }
}

void validateEqualityReduction(
    const std::vector<NewtonEqualityReductionEntry>& entries,
    const std::size_t fullDofCount,
    const std::size_t reducedDofCount)
{
  DART_SIMULATION_THROW_T_IF(
      reducedDofCount == 0 && !entries.empty(),
      sx::InvalidArgumentException,
      "evaluateNewtonEqualityReducedSolveCuda requires reduced dofs for "
      "reduction entries");

  for (std::size_t row = 0; row < entries.size(); ++row) {
    const auto& entry = entries[row];
    DART_SIMULATION_THROW_T_IF(
        entry.fullDofIndex >= fullDofCount,
        sx::InvalidArgumentException,
        "evaluateNewtonEqualityReducedSolveCuda entry {} full dof {} is "
        "outside {} dofs",
        row,
        entry.fullDofIndex,
        fullDofCount);
    DART_SIMULATION_THROW_T_IF(
        entry.reducedDofIndex >= reducedDofCount,
        sx::InvalidArgumentException,
        "evaluateNewtonEqualityReducedSolveCuda entry {} reduced dof {} is "
        "outside {} reduced dofs",
        row,
        entry.reducedDofIndex,
        reducedDofCount);
    DART_SIMULATION_THROW_T_IF(
        !std::isfinite(entry.basisValue) || entry.basisValue == 0.0,
        sx::InvalidArgumentException,
        "evaluateNewtonEqualityReducedSolveCuda entry {} has an invalid "
        "basis value",
        row);
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

//==============================================================================
void evaluateNewtonSparseResidualCuda(
    const std::vector<NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const std::vector<NewtonSparseBlockEntry>& blocks,
    const std::vector<double>& step,
    const double regularization,
    NewtonSparseResidualResult& result)
{
  const auto setupStart = Clock::now();
  validateRows(rows, bodyCount, regularization);
  validateSparseBlocks(blocks, bodyCount);
  const std::size_t dofCount = bodyCount * kNewtonAssemblySolveDofsPerBody;
  validateStep(step, dofCount);

  result = NewtonSparseResidualResult{};
  result.bodyCount = bodyCount;
  result.rowCount = rows.size();
  result.dofCount = dofCount;
  result.blockCount = blocks.size();
  result.assembledDiagonal.assign(dofCount, 0.0);
  result.assembledGradient.assign(dofCount, 0.0);
  result.residual.assign(dofCount, 0.0);

  if (dofCount == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<NewtonAssemblySolveRowInput> deviceRows(rows.size());
  DeviceBuffer<NewtonSparseBlockEntry> deviceBlocks(blocks.size());
  DeviceBuffer<double> deviceStep(dofCount);
  DeviceBuffer<double> deviceDiagonal(dofCount);
  DeviceBuffer<double> deviceGradient(dofCount);
  DeviceBuffer<double> deviceResidual(dofCount);
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceRows.copyToDevice(rows, "newton sparse residual rows copy");
  deviceBlocks.copyToDevice(blocks, "newton sparse residual blocks copy");
  deviceStep.copyToDevice(step, "newton sparse residual step copy");
  throwIfCudaError(
      cudaMemset(deviceDiagonal.data(), 0, deviceDiagonal.byteSize()),
      "newton sparse residual diagonal zero");
  throwIfCudaError(
      cudaMemset(deviceGradient.data(), 0, deviceGradient.byteSize()),
      "newton sparse residual gradient zero");
  throwIfCudaError(
      cudaMemset(deviceResidual.data(), 0, deviceResidual.byteSize()),
      "newton sparse residual output zero");
  const auto h2dEnd = Clock::now();

  const auto assemblyStart = Clock::now();
  throwIfCudaError(
      detail::launchNewtonAssemblyRowsKernel(
          deviceRows.data(),
          deviceDiagonal.data(),
          deviceGradient.data(),
          rows.size()),
      "newton sparse residual assembly rows kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(),
      "newton sparse residual assembly rows synchronize");
  const auto assemblyEnd = Clock::now();

  const auto gradientSeedStart = Clock::now();
  throwIfCudaError(
      cudaMemcpy(
          deviceResidual.data(),
          deviceGradient.data(),
          deviceResidual.byteSize(),
          cudaMemcpyDeviceToDevice),
      "newton sparse residual gradient seed");
  throwIfCudaError(
      cudaDeviceSynchronize(), "newton sparse residual gradient synchronize");
  const auto gradientSeedEnd = Clock::now();

  const auto diagonalStart = Clock::now();
  throwIfCudaError(
      detail::launchNewtonSparseDiagonalResidualKernel(
          deviceDiagonal.data(),
          deviceStep.data(),
          deviceResidual.data(),
          dofCount,
          regularization),
      "newton sparse residual diagonal kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "newton sparse residual diagonal synchronize");
  const auto diagonalEnd = Clock::now();

  const auto offDiagonalStart = Clock::now();
  throwIfCudaError(
      detail::launchNewtonSparseBlockResidualKernel(
          deviceBlocks.data(),
          deviceStep.data(),
          deviceResidual.data(),
          blocks.size()),
      "newton sparse residual block kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "newton sparse residual block synchronize");
  const auto offDiagonalEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceDiagonal.copyFromDevice(
      result.assembledDiagonal, "newton sparse residual diagonal copy");
  deviceGradient.copyFromDevice(
      result.assembledGradient, "newton sparse residual gradient copy");
  deviceResidual.copyFromDevice(
      result.residual, "newton sparse residual output copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.assemblyKernelNs = elapsedNs(assemblyStart, assemblyEnd);
  result.timing.gradientSeedNs = elapsedNs(gradientSeedStart, gradientSeedEnd);
  result.timing.diagonalKernelNs = elapsedNs(diagonalStart, diagonalEnd);
  result.timing.offDiagonalKernelNs
      = elapsedNs(offDiagonalStart, offDiagonalEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  double residualNormSquared = 0.0;
  for (std::size_t dof = 0; dof < dofCount; ++dof) {
    result.maxDiagonal
        = std::max(result.maxDiagonal, result.assembledDiagonal[dof]);
    result.maxGradientAbs = std::max(
        result.maxGradientAbs, std::abs(result.assembledGradient[dof]));
    const double residualAbs = std::abs(result.residual[dof]);
    result.maxResidualAbs = std::max(result.maxResidualAbs, residualAbs);
    if (residualAbs > 0.0) {
      ++result.activeDofCount;
    }
    residualNormSquared += result.residual[dof] * result.residual[dof];
  }
  result.residualNorm = std::sqrt(residualNormSquared);
}

//==============================================================================
void evaluateNewtonEqualityReducedSolveCuda(
    const std::vector<NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    const std::vector<NewtonEqualityReductionEntry>& reductionEntries,
    const std::size_t reducedDofCount,
    const double regularization,
    NewtonEqualityReducedSolveResult& result)
{
  const auto setupStart = Clock::now();
  validateRows(rows, bodyCount, regularization);
  const std::size_t fullDofCount = bodyCount * kNewtonAssemblySolveDofsPerBody;
  validateEqualityReduction(reductionEntries, fullDofCount, reducedDofCount);

  result = NewtonEqualityReducedSolveResult{};
  result.bodyCount = bodyCount;
  result.rowCount = rows.size();
  result.fullDofCount = fullDofCount;
  result.reductionEntryCount = reductionEntries.size();
  result.reducedDofCount = reducedDofCount;
  result.assembledDiagonal.assign(fullDofCount, 0.0);
  result.assembledGradient.assign(fullDofCount, 0.0);
  result.reducedDiagonal.assign(reducedDofCount, 0.0);
  result.reducedGradient.assign(reducedDofCount, 0.0);
  result.reducedStep.assign(reducedDofCount, 0.0);
  result.reducedResidual.assign(reducedDofCount, 0.0);
  result.fullStep.assign(fullDofCount, 0.0);

  if (fullDofCount == 0 || reducedDofCount == 0) {
    return;
  }

  throwIfCudaRuntimeUnavailable();

  DeviceBuffer<NewtonAssemblySolveRowInput> deviceRows(rows.size());
  DeviceBuffer<NewtonEqualityReductionEntry> deviceEntries(
      reductionEntries.size());
  DeviceBuffer<double> deviceDiagonal(fullDofCount);
  DeviceBuffer<double> deviceGradient(fullDofCount);
  DeviceBuffer<double> deviceReducedDiagonal(reducedDofCount);
  DeviceBuffer<double> deviceReducedGradient(reducedDofCount);
  DeviceBuffer<double> deviceReducedStep(reducedDofCount);
  DeviceBuffer<double> deviceReducedResidual(reducedDofCount);
  DeviceBuffer<double> deviceFullStep(fullDofCount);
  const auto setupEnd = Clock::now();

  const auto h2dStart = Clock::now();
  deviceRows.copyToDevice(rows, "newton equality assembly rows copy");
  deviceEntries.copyToDevice(
      reductionEntries, "newton equality reduction entries copy");
  throwIfCudaError(
      cudaMemset(deviceDiagonal.data(), 0, deviceDiagonal.byteSize()),
      "newton equality assembly diagonal zero");
  throwIfCudaError(
      cudaMemset(deviceGradient.data(), 0, deviceGradient.byteSize()),
      "newton equality assembly gradient zero");
  throwIfCudaError(
      cudaMemset(
          deviceReducedDiagonal.data(), 0, deviceReducedDiagonal.byteSize()),
      "newton equality reduced diagonal zero");
  throwIfCudaError(
      cudaMemset(
          deviceReducedGradient.data(), 0, deviceReducedGradient.byteSize()),
      "newton equality reduced gradient zero");
  throwIfCudaError(
      cudaMemset(deviceFullStep.data(), 0, deviceFullStep.byteSize()),
      "newton equality full step zero");
  const auto h2dEnd = Clock::now();

  const auto assemblyStart = Clock::now();
  throwIfCudaError(
      detail::launchNewtonAssemblyRowsKernel(
          deviceRows.data(),
          deviceDiagonal.data(),
          deviceGradient.data(),
          rows.size()),
      "newton equality assembly rows kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "newton equality assembly rows synchronize");
  const auto assemblyEnd = Clock::now();

  const auto reductionStart = Clock::now();
  throwIfCudaError(
      detail::launchNewtonEqualityReductionKernel(
          deviceEntries.data(),
          deviceDiagonal.data(),
          deviceGradient.data(),
          deviceReducedDiagonal.data(),
          deviceReducedGradient.data(),
          reductionEntries.size()),
      "newton equality reduction kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "newton equality reduction synchronize");
  const auto reductionEnd = Clock::now();

  const auto solveStart = Clock::now();
  throwIfCudaError(
      detail::launchNewtonDiagonalSolveKernel(
          deviceReducedDiagonal.data(),
          deviceReducedGradient.data(),
          deviceReducedStep.data(),
          deviceReducedResidual.data(),
          reducedDofCount,
          regularization,
          1e-14),
      "newton equality reduced solve kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "newton equality reduced solve synchronize");
  const auto solveEnd = Clock::now();

  const auto expansionStart = Clock::now();
  throwIfCudaError(
      detail::launchNewtonExpandEqualityReducedStepKernel(
          deviceEntries.data(),
          deviceReducedStep.data(),
          deviceFullStep.data(),
          reductionEntries.size()),
      "newton equality step expansion kernel");
  throwIfCudaError(
      cudaDeviceSynchronize(), "newton equality step expansion synchronize");
  const auto expansionEnd = Clock::now();

  const auto d2hStart = Clock::now();
  deviceDiagonal.copyFromDevice(
      result.assembledDiagonal, "newton equality assembled diagonal copy");
  deviceGradient.copyFromDevice(
      result.assembledGradient, "newton equality assembled gradient copy");
  deviceReducedDiagonal.copyFromDevice(
      result.reducedDiagonal, "newton equality reduced diagonal copy");
  deviceReducedGradient.copyFromDevice(
      result.reducedGradient, "newton equality reduced gradient copy");
  deviceReducedStep.copyFromDevice(
      result.reducedStep, "newton equality reduced step copy");
  deviceReducedResidual.copyFromDevice(
      result.reducedResidual, "newton equality reduced residual copy");
  deviceFullStep.copyFromDevice(result.fullStep, "newton equality step copy");
  const auto d2hEnd = Clock::now();

  result.timing.setupNs = elapsedNs(setupStart, setupEnd);
  result.timing.hostToDeviceNs = elapsedNs(h2dStart, h2dEnd);
  result.timing.assemblyKernelNs = elapsedNs(assemblyStart, assemblyEnd);
  result.timing.reductionKernelNs = elapsedNs(reductionStart, reductionEnd);
  result.timing.solveKernelNs = elapsedNs(solveStart, solveEnd);
  result.timing.expansionKernelNs = elapsedNs(expansionStart, expansionEnd);
  result.timing.deviceToHostNs = elapsedNs(d2hStart, d2hEnd);

  double stepNormSquared = 0.0;
  double residualNormSquared = 0.0;
  for (std::size_t dof = 0; dof < reducedDofCount; ++dof) {
    const double effectiveDiagonal
        = result.reducedDiagonal[dof] + regularization;
    if (effectiveDiagonal > 1e-14) {
      ++result.activeReducedDofCount;
    }
    result.maxDiagonal
        = std::max(result.maxDiagonal, result.reducedDiagonal[dof]);
    result.maxGradientAbs = std::max(
        result.maxGradientAbs, std::abs(result.reducedGradient[dof]));
    stepNormSquared += result.reducedStep[dof] * result.reducedStep[dof];
    residualNormSquared
        += result.reducedResidual[dof] * result.reducedResidual[dof];
  }
  result.stepNorm = std::sqrt(stepNormSquared);
  result.residualNorm = std::sqrt(residualNormSquared);
}

} // namespace dart::simulation::compute::cuda
