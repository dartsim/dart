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

#include <algorithm>

#include <cstddef>

namespace dart::simulation::detail::deformable_vbd {

/// Minimum useful scalar AVBD row inventory for task-parallel dispatch. A
/// matched hard-contact-row benchmark keeps smaller inventories inline because
/// waking the persistent worker pool costs more than the row arithmetic.
inline constexpr std::size_t kAvbdParallelRowUpdateMinCount = 8192u;

/// Minimum range assigned to one worker after the dispatch cost gate passes.
inline constexpr std::size_t kAvbdParallelRowUpdateMinGrain = 2048u;

//==============================================================================
/// Visit deterministic contiguous ranges of an AVBD row family. The callback
/// must update only rows in [begin, end); the executor call is synchronous, so
/// all row writes are complete before the next primal sweep begins.
template <typename Function>
inline void forEachAvbdRowUpdateRange(
    compute::ComputeExecutor* executor, std::size_t count, Function&& function)
{
  if (count == 0u) {
    return;
  }

  const std::size_t workerCount
      = executor != nullptr ? executor->getWorkerCount() : 0u;
  if (workerCount <= 1u || count <= kAvbdParallelRowUpdateMinCount) {
    function(0u, count);
    return;
  }

  const std::size_t balancedGrain = 1u + (count - 1u) / workerCount;
  executor->parallelFor(
      count, std::max(kAvbdParallelRowUpdateMinGrain, balancedGrain), function);
}

} // namespace dart::simulation::detail::deformable_vbd
