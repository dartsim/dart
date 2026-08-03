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

// Isolates the AVBD Section 4 post-sweep dual/stiffness pass. The zero-worker
// cases are the exact serial reference; the other cases exercise the same
// deterministic chunk policy used by the solver drivers.

#include <dart/simulation/compute/compute_executor.hpp>
#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/detail/deformable_vbd/contact_kernel.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <memory>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace compute = dart::simulation::compute;
namespace vbd = dart::simulation::detail::deformable_vbd;

namespace {

inline constexpr std::size_t kRowUpdateMinCount = 8192u;
inline constexpr std::size_t kRowUpdateMinGrain = 2048u;

template <typename Function>
void forEachRowUpdateRange(
    compute::ComputeExecutor* executor, std::size_t count, Function&& function)
{
  const std::size_t workerCount
      = executor != nullptr ? executor->getWorkerCount() : 0u;
  if (workerCount <= 1u || count <= kRowUpdateMinCount) {
    function(0u, count);
    return;
  }

  const std::size_t balancedGrain = 1u + (count - 1u) / workerCount;
  executor->parallelFor(
      count, std::max(kRowUpdateMinGrain, balancedGrain), function);
}

} // namespace

//==============================================================================
static void BM_AvbdContactDualUpdatePass(benchmark::State& state)
{
  const std::size_t rowCount = static_cast<std::size_t>(state.range(0));
  const std::size_t workerCount = static_cast<std::size_t>(state.range(1));

  std::vector<Eigen::Vector3d> positions(rowCount);
  std::vector<vbd::AvbdHalfSpaceContactRow> rows(rowCount);
  for (std::size_t rowIndex = 0u; rowIndex < rowCount; ++rowIndex) {
    const double offset = 1.0e-4 * static_cast<double>(rowIndex % 17u);
    positions[rowIndex] = Eigen::Vector3d(0.01, 0.001 + offset, -0.02);
    auto& row = rows[rowIndex];
    row.vertex = static_cast<std::uint32_t>(rowIndex);
    row.plane.normal = Eigen::Vector3d::UnitY();
    row.state.lambda = 0.25;
    row.state.stiffness = 10.0 + static_cast<double>(rowIndex % 5u);
    row.previousConstraintValue = 0.5 * offset;
  }

  vbd::AvbdHalfSpaceContactOptions options;
  options.alpha = 0.25;
  options.beta = 0.5;
  options.maxStiffness = 1.0e8;

  std::unique_ptr<compute::ParallelExecutor> parallelExecutor;
  if (workerCount != 0u) {
    parallelExecutor = std::make_unique<compute::ParallelExecutor>(workerCount);
  }
  compute::ComputeExecutor* const executor = parallelExecutor.get();
  const auto updateRows = [&]() {
    forEachRowUpdateRange(
        executor, rowCount, [&](std::size_t begin, std::size_t end) {
          for (std::size_t rowIndex = begin; rowIndex < end; ++rowIndex) {
            auto& row = rows[rowIndex];
            row.state = vbd::updateAvbdHalfSpaceContactNormalRow(
                row.state,
                positions[row.vertex],
                row.plane,
                options,
                row.previousConstraintValue,
                row.bounds);
          }
        });
  };

  updateRows();
  updateRows();
  for (auto _ : state) {
    updateRows();
    benchmark::DoNotOptimize(rows.data());
    benchmark::ClobberMemory();
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<std::int64_t>(rowCount));
  state.counters["rows"] = static_cast<double>(rowCount);
  state.counters["workers"] = static_cast<double>(workerCount);
}
BENCHMARK(BM_AvbdContactDualUpdatePass)
    ->ArgsProduct({{256, 4096, 8192, 16384, 32768, 65536}, {0, 2, 4}})
    ->ArgNames({"rows", "workers"})
    ->UseRealTime();

BENCHMARK_MAIN();
