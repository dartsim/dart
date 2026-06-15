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

#pragma once

#include <dart/simulation/compute/compute_executor.hpp>

#include <memory>

#include <cstddef>

namespace dart::simulation::compute {

/// Experimental executor that runs ready graph nodes in parallel.
class DART_SIMULATION_API ParallelExecutor final : public ComputeExecutor
{
public:
  explicit ParallelExecutor(std::size_t workerCount = 0);
  ~ParallelExecutor() override;

  ParallelExecutor(const ParallelExecutor&) = delete;
  ParallelExecutor& operator=(const ParallelExecutor&) = delete;
  ParallelExecutor(ParallelExecutor&&) noexcept;
  ParallelExecutor& operator=(ParallelExecutor&&) noexcept;

  void execute(const ComputeGraph& graph) override;
  [[nodiscard]] ComputeExecutionProfile executeProfiled(
      const ComputeGraph& graph) override;
  void executeProfiled(
      const ComputeGraph& graph, ComputeExecutionProfile& profile) override;
  [[nodiscard]] std::size_t getWorkerCount() const override;

  /// Cost gate: graphs with at most this many nodes execute inline
  /// (sequentially, in topological order) instead of building a Taskflow, since
  /// scheduling overhead dominates when there is little or no parallelism to
  /// exploit. The inline path is the sequential reference order, so results are
  /// unchanged. Defaults to 1 (a single-node graph has no parallelism);
  /// profiling can raise it to coarsen the inline/parallel boundary.
  void setInlineThreshold(std::size_t threshold) noexcept;
  [[nodiscard]] std::size_t getInlineThreshold() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace dart::simulation::compute
