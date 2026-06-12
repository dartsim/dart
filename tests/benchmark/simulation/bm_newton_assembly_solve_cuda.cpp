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

#include <benchmark/benchmark.h>
#include <dart/simulation/compute/cuda/cuda_runtime.cuh>
#include <dart/simulation/compute/cuda/newton_assembly_solve_cuda.cuh>

#include <algorithm>
#include <vector>

#include <cmath>
#include <cstdint>

namespace cuda = dart::simulation::compute::cuda;

namespace {

constexpr double kRegularization = 0.25;

struct CpuAssemblySolveResult
{
  std::vector<double> assembledDiagonal;
  std::vector<double> assembledGradient;
  std::vector<double> step;
  std::vector<double> residual;
  std::size_t bodyCount = 0;
  std::size_t rowCount = 0;
  std::size_t activeDofCount = 0;
  double maxDiagonal = 0.0;
  double maxGradientAbs = 0.0;
  double stepNorm = 0.0;
  double residualNorm = 0.0;
};

struct AssemblySolveFixture
{
  std::vector<cuda::NewtonAssemblySolveRowInput> rows;
  std::size_t bodyCount = 0;
  CpuAssemblySolveResult cpu;
};

struct CpuOffDiagonalAssemblyResult
{
  std::vector<double> assembledBlocks;
  std::size_t pairCount = 0;
  std::size_t rowCount = 0;
  std::size_t activeBlockCount = 0;
  double maxBlockAbs = 0.0;
};

struct OffDiagonalAssemblyFixture
{
  std::vector<cuda::NewtonOffDiagonalAssemblyRowInput> rows;
  std::size_t pairCount = 0;
  CpuOffDiagonalAssemblyResult cpu;
};

cuda::NewtonAssemblySolveRowInput makeRow(
    const int rowIndex, const std::size_t bodyCount)
{
  cuda::NewtonAssemblySolveRowInput row;
  row.bodyIndex = static_cast<std::uint32_t>(
      static_cast<std::size_t>(rowIndex) % bodyCount);
  for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
       ++dof) {
    const auto signedBucket = static_cast<int>((rowIndex + 3 * dof) % 23) - 11;
    row.hessianDiagonal[dof]
        = 0.5 + 0.01 * static_cast<double>((rowIndex + dof) % 17)
          + 0.05 * static_cast<double>(dof + 1);
    row.gradient[dof] = 0.025 * static_cast<double>(signedBucket);
  }
  return row;
}

cuda::NewtonOffDiagonalAssemblyRowInput makeOffDiagonalRow(
    const int rowIndex, const std::size_t pairCount)
{
  cuda::NewtonOffDiagonalAssemblyRowInput row;
  row.pairIndex = static_cast<std::uint32_t>(
      static_cast<std::size_t>(rowIndex) % pairCount);
  for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
       ++entry) {
    const auto signedBucket
        = static_cast<int>((rowIndex + 7 * entry) % 31) - 15;
    row.hessianBlock[entry] = 0.0025 * static_cast<double>(signedBucket)
                              + 0.0001 * static_cast<double>((entry % 6) + 1);
  }
  return row;
}

void evaluateCpu(
    const std::vector<cuda::NewtonAssemblySolveRowInput>& rows,
    const std::size_t bodyCount,
    CpuAssemblySolveResult& result)
{
  const std::size_t dofCount
      = bodyCount * cuda::kNewtonAssemblySolveDofsPerBody;
  result = CpuAssemblySolveResult{};
  result.bodyCount = bodyCount;
  result.rowCount = rows.size();
  result.assembledDiagonal.assign(dofCount, 0.0);
  result.assembledGradient.assign(dofCount, 0.0);
  result.step.assign(dofCount, 0.0);
  result.residual.assign(dofCount, 0.0);

  for (const auto& row : rows) {
    const std::size_t offset = static_cast<std::size_t>(row.bodyIndex)
                               * cuda::kNewtonAssemblySolveDofsPerBody;
    for (std::size_t dof = 0; dof < cuda::kNewtonAssemblySolveDofsPerBody;
         ++dof) {
      result.assembledDiagonal[offset + dof] += row.hessianDiagonal[dof];
      result.assembledGradient[offset + dof] += row.gradient[dof];
    }
  }

  double stepNormSquared = 0.0;
  double residualNormSquared = 0.0;
  for (std::size_t dof = 0; dof < dofCount; ++dof) {
    const double effectiveDiagonal
        = result.assembledDiagonal[dof] + kRegularization;
    if (effectiveDiagonal > 1e-14) {
      ++result.activeDofCount;
    }
    result.step[dof] = -result.assembledGradient[dof] / effectiveDiagonal;
    result.residual[dof]
        = effectiveDiagonal * result.step[dof] + result.assembledGradient[dof];
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

void evaluateCpuOffDiagonalAssembly(
    const std::vector<cuda::NewtonOffDiagonalAssemblyRowInput>& rows,
    const std::size_t pairCount,
    CpuOffDiagonalAssemblyResult& result)
{
  result = CpuOffDiagonalAssemblyResult{};
  result.pairCount = pairCount;
  result.rowCount = rows.size();
  result.assembledBlocks.assign(
      pairCount * cuda::kNewtonAssemblySolveBlockEntries, 0.0);

  for (const auto& row : rows) {
    const std::size_t offset = static_cast<std::size_t>(row.pairIndex)
                               * cuda::kNewtonAssemblySolveBlockEntries;
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
         ++entry) {
      result.assembledBlocks[offset + entry] += row.hessianBlock[entry];
    }
  }

  for (std::size_t pair = 0; pair < pairCount; ++pair) {
    bool active = false;
    const std::size_t offset = pair * cuda::kNewtonAssemblySolveBlockEntries;
    for (std::size_t entry = 0; entry < cuda::kNewtonAssemblySolveBlockEntries;
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

AssemblySolveFixture makeFixture(const int rowCount)
{
  AssemblySolveFixture fixture;
  fixture.bodyCount
      = std::max<std::size_t>(1, static_cast<std::size_t>(rowCount) / 8);
  fixture.rows.reserve(static_cast<std::size_t>(rowCount));
  for (int row = 0; row < rowCount; ++row) {
    fixture.rows.push_back(makeRow(row, fixture.bodyCount));
  }
  evaluateCpu(fixture.rows, fixture.bodyCount, fixture.cpu);
  return fixture;
}

OffDiagonalAssemblyFixture makeOffDiagonalFixture(const int rowCount)
{
  OffDiagonalAssemblyFixture fixture;
  fixture.pairCount
      = std::max<std::size_t>(1, static_cast<std::size_t>(rowCount) / 16);
  fixture.rows.reserve(static_cast<std::size_t>(rowCount));
  for (int row = 0; row < rowCount; ++row) {
    fixture.rows.push_back(makeOffDiagonalRow(row, fixture.pairCount));
  }
  evaluateCpuOffDiagonalAssembly(fixture.rows, fixture.pairCount, fixture.cpu);
  return fixture;
}

template <typename Lhs, typename Rhs>
double maxAbsDifference(const Lhs& lhs, const Rhs& rhs)
{
  double maxError = 0.0;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    maxError = std::max(maxError, std::abs(lhs[i] - rhs[i]));
  }
  return maxError;
}

double maxOutputError(
    const CpuAssemblySolveResult& expected,
    const cuda::NewtonAssemblySolveResult& actual)
{
  double maxError = 0.0;
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledDiagonal, actual.assembledDiagonal));
  maxError = std::max(
      maxError,
      maxAbsDifference(expected.assembledGradient, actual.assembledGradient));
  maxError = std::max(maxError, maxAbsDifference(expected.step, actual.step));
  maxError = std::max(
      maxError, maxAbsDifference(expected.residual, actual.residual));
  return maxError;
}

double maxOffDiagonalOutputError(
    const CpuOffDiagonalAssemblyResult& expected,
    const cuda::NewtonOffDiagonalAssemblyResult& actual)
{
  return maxAbsDifference(expected.assembledBlocks, actual.assembledBlocks);
}

void recordCounters(
    benchmark::State& state,
    const AssemblySolveFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.rows.size());
  state.counters["bodies"] = static_cast<double>(fixture.bodyCount);
  state.counters["dofs"] = static_cast<double>(
      fixture.bodyCount * cuda::kNewtonAssemblySolveDofsPerBody);
  state.counters["active_dofs"]
      = static_cast<double>(fixture.cpu.activeDofCount);
  state.counters["max_diagonal"] = fixture.cpu.maxDiagonal;
  state.counters["max_gradient_abs"] = fixture.cpu.maxGradientAbs;
  state.counters["step_norm"] = fixture.cpu.stepNorm;
  state.counters["residual_norm"] = fixture.cpu.residualNorm;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.rows.size()));
}

void recordOffDiagonalCounters(
    benchmark::State& state,
    const OffDiagonalAssemblyFixture& fixture,
    const double maxError)
{
  state.counters["rows"] = static_cast<double>(fixture.rows.size());
  state.counters["pairs"] = static_cast<double>(fixture.pairCount);
  state.counters["block_entries"] = static_cast<double>(
      fixture.pairCount * cuda::kNewtonAssemblySolveBlockEntries);
  state.counters["active_blocks"]
      = static_cast<double>(fixture.cpu.activeBlockCount);
  state.counters["max_block_abs"] = fixture.cpu.maxBlockAbs;
  state.counters["max_result_abs_error"] = maxError;
  state.SetItemsProcessed(
      static_cast<std::int64_t>(state.iterations() * fixture.rows.size()));
}

} // namespace

//==============================================================================
static void BM_NewtonAssemblySolveCpu(benchmark::State& state)
{
  const auto fixture = makeFixture(static_cast<int>(state.range(0)));
  CpuAssemblySolveResult result;

  for (auto _ : state) {
    evaluateCpu(fixture.rows, fixture.bodyCount, result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonAssemblySolveCpu)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_NewtonAssemblySolveCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeFixture(static_cast<int>(state.range(0)));
  cuda::NewtonAssemblySolveResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonAssemblySolveCuda(
        fixture.rows, fixture.bodyCount, kRegularization, result);
    benchmark::DoNotOptimize(result.step.data());
  }

  recordCounters(state, fixture, maxOutputError(fixture.cpu, result));
  state.counters["gpu_bodies"] = static_cast<double>(result.bodyCount);
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_active_dofs"]
      = static_cast<double>(result.activeDofCount);
  state.counters["gpu_step_norm"] = result.stepNorm;
  state.counters["gpu_residual_norm"] = result.residualNorm;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["solve_kernel_ns"] = result.timing.solveKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonAssemblySolveCuda)->Arg(4096)->Arg(65536)->UseRealTime();

//==============================================================================
static void BM_NewtonOffDiagonalAssemblyCpu(benchmark::State& state)
{
  const auto fixture = makeOffDiagonalFixture(static_cast<int>(state.range(0)));
  CpuOffDiagonalAssemblyResult result;

  for (auto _ : state) {
    evaluateCpuOffDiagonalAssembly(fixture.rows, fixture.pairCount, result);
    benchmark::DoNotOptimize(result.assembledBlocks.data());
  }

  recordOffDiagonalCounters(state, fixture, 0.0);
}
BENCHMARK(BM_NewtonOffDiagonalAssemblyCpu)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

//==============================================================================
static void BM_NewtonOffDiagonalAssemblyCuda(benchmark::State& state)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    state.SkipWithError("CUDA runtime has no available device");
    return;
  }

  const auto fixture = makeOffDiagonalFixture(static_cast<int>(state.range(0)));
  cuda::NewtonOffDiagonalAssemblyResult result;

  for (auto _ : state) {
    cuda::evaluateNewtonOffDiagonalAssemblyCuda(
        fixture.rows, fixture.pairCount, result);
    benchmark::DoNotOptimize(result.assembledBlocks.data());
  }

  recordOffDiagonalCounters(
      state, fixture, maxOffDiagonalOutputError(fixture.cpu, result));
  state.counters["gpu_rows"] = static_cast<double>(result.rowCount);
  state.counters["gpu_pairs"] = static_cast<double>(result.pairCount);
  state.counters["gpu_active_blocks"]
      = static_cast<double>(result.activeBlockCount);
  state.counters["gpu_max_block_abs"] = result.maxBlockAbs;
  state.counters["host_setup_ns"] = result.timing.setupNs;
  state.counters["host_to_device_ns"] = result.timing.hostToDeviceNs;
  state.counters["assembly_kernel_ns"] = result.timing.assemblyKernelNs;
  state.counters["solve_kernel_ns"] = result.timing.solveKernelNs;
  state.counters["device_to_host_ns"] = result.timing.deviceToHostNs;
}
BENCHMARK(BM_NewtonOffDiagonalAssemblyCuda)
    ->Arg(4096)
    ->Arg(65536)
    ->UseRealTime();

BENCHMARK_MAIN();
