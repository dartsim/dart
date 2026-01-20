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

#include <dart/simulation/compute_graph/compute_graph.hpp>

#include <future>
#include <memory>

namespace dart::simulation {

/// Abstract interface for graph executors.
/// Allows swapping backends (Taskflow, TBB, custom) without changing graph
/// construction.
class DART_API GraphExecutor
{
public:
  virtual ~GraphExecutor() = default;

  virtual void execute(ComputeGraph& graph, const ExecutionContext& ctx) = 0;

  virtual std::future<void> executeAsync(
      ComputeGraph& graph, const ExecutionContext& ctx)
      = 0;

  [[nodiscard]] virtual std::size_t getNumWorkers() const = 0;

  virtual void waitAll() = 0;

  [[nodiscard]] static GraphExecutorPtr create(
      const ExecutorConfig& config = {});
};

/// Executes nodes sequentially in topological order.
/// Useful for debugging and determinism verification.
class DART_API SequentialExecutor : public GraphExecutor
{
public:
  void execute(ComputeGraph& graph, const ExecutionContext& ctx) override;

  std::future<void> executeAsync(
      ComputeGraph& graph, const ExecutionContext& ctx) override;

  [[nodiscard]] std::size_t getNumWorkers() const override
  {
    return 1;
  }

  void waitAll() override {}
};

} // namespace dart::simulation
