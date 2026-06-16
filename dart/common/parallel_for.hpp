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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
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

#include <algorithm>
#include <functional>
#include <thread>
#include <vector>

#include <cstddef>

namespace dart::common {

using ParallelForChunkFunction = std::function<void(
    std::size_t begin, std::size_t end, std::size_t chunkIndex)>;

inline std::size_t resolveParallelForWorkerCount(
    std::size_t maxWorkerCount) noexcept
{
  const unsigned int hardware = std::thread::hardware_concurrency();
  if (maxWorkerCount == 0u) {
    return std::max<std::size_t>(1u, hardware);
  }
  if (hardware > 0u) {
    maxWorkerCount = std::min<std::size_t>(maxWorkerCount, hardware);
  }
  return std::max<std::size_t>(1u, maxWorkerCount);
}

inline std::size_t parallelForChunkSize(
    std::size_t count, std::size_t grainSize, std::size_t maxWorkerCount)
{
  if (count == 0u) {
    return 0u;
  }
  const std::size_t workerCount = resolveParallelForWorkerCount(maxWorkerCount);
  grainSize = std::max<std::size_t>(1u, grainSize);
  return std::max(grainSize, (count + workerCount - 1u) / workerCount);
}

inline std::size_t parallelForChunkCount(
    std::size_t count, std::size_t grainSize, std::size_t maxWorkerCount)
{
  const std::size_t chunkSize
      = parallelForChunkSize(count, grainSize, maxWorkerCount);
  return chunkSize == 0u ? 0u : (count + chunkSize - 1u) / chunkSize;
}

inline void parallelForChunks(
    std::size_t count,
    std::size_t grainSize,
    std::size_t maxWorkerCount,
    const ParallelForChunkFunction& function)
{
  const std::size_t chunkSize
      = parallelForChunkSize(count, grainSize, maxWorkerCount);
  if (chunkSize == 0u) {
    return;
  }
  const std::size_t chunkCount = (count + chunkSize - 1u) / chunkSize;
  if (chunkCount <= 1u) {
    function(0u, count, 0u);
    return;
  }

  std::vector<std::thread> threads;
  threads.reserve(chunkCount);
  for (std::size_t chunkIndex = 0u; chunkIndex < chunkCount; ++chunkIndex) {
    const std::size_t begin = chunkIndex * chunkSize;
    const std::size_t end = std::min(begin + chunkSize, count);
    if (begin >= end) {
      continue;
    }
    threads.emplace_back([begin, end, chunkIndex, &function]() {
      function(begin, end, chunkIndex);
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }
}

} // namespace dart::common
