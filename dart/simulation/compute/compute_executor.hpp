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

#include <dart/simulation/compute/execution_profile.hpp>
#include <dart/simulation/export.hpp>

#include <cstddef>

namespace dart::simulation::compute {

class ComputeGraph;

/// Backend-neutral interface for executing experimental compute graphs.
///
/// Concurrency contract for parallel backends. A node callable may run on any
/// worker thread, so graphs must satisfy these invariants for parallel
/// execution to be safe and deterministic:
///   - no node may create, destroy, add, or remove entities or components while
///     the graph executes (no structural changes mid-execution);
///   - two nodes that are not ordered by an explicit dependency must not write
///     the same component instance; per-entity work should use per-entity
///     resource identifiers so independent siblings stay conflict-free;
///   - explicit dependencies remain the correctness source of truth; declared
///     resource-access metadata is a descriptive check layered on top of them
///     (see @c ComputeGraph::findResourceHazards).
/// Sequential execution is the reference path and is always safe; the parallel
/// backend additionally checks declared accesses for unordered hazards in debug
/// builds.
class DART_SIMULATION_API ComputeExecutor
{
public:
  virtual ~ComputeExecutor() = default;

  ComputeExecutor() = default;
  ComputeExecutor(const ComputeExecutor&) = delete;
  ComputeExecutor& operator=(const ComputeExecutor&) = delete;
  ComputeExecutor(ComputeExecutor&&) noexcept = default;
  ComputeExecutor& operator=(ComputeExecutor&&) noexcept = default;

  virtual void execute(const ComputeGraph& graph) = 0;
  [[nodiscard]] virtual ComputeExecutionProfile executeProfiled(
      const ComputeGraph& graph) = 0;
  virtual void executeProfiled(
      const ComputeGraph& graph, ComputeExecutionProfile& profile)
  {
    profile = executeProfiled(graph);
  }
  [[nodiscard]] virtual std::size_t getWorkerCount() const = 0;
};

} // namespace dart::simulation::compute
