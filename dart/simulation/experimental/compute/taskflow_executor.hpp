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

#include <dart/simulation/experimental/compute/compute_executor.hpp>

#include <memory>

namespace tf {
class Executor;
}

namespace dart::simulation::experimental::compute {

/// @brief Parallel executor backed by Taskflow.
///
/// @param numThreads Number of worker threads. 0 means use hardware
/// concurrency.
class DART_EXPERIMENTAL_API TaskflowExecutor : public ComputeExecutor
{
public:
  explicit TaskflowExecutor(std::size_t numThreads = 0);
  ~TaskflowExecutor() override;

  TaskflowExecutor(const TaskflowExecutor&) = delete;
  TaskflowExecutor& operator=(const TaskflowExecutor&) = delete;
  TaskflowExecutor(TaskflowExecutor&&) noexcept;
  TaskflowExecutor& operator=(TaskflowExecutor&&) noexcept;

  void execute(const ComputeGraph& graph) override;

  [[nodiscard]] std::size_t getWorkerCount() const override;

private:
  std::unique_ptr<tf::Executor> m_executor;
};

} // namespace dart::simulation::experimental::compute
