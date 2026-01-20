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

#include <dart/simulation/compute_graph/fwd.hpp>

#include <functional>
#include <string>
#include <string_view>

namespace dart::simulation {

/// Base class for all compute graph nodes.
/// Thread-safety: Nodes must be thread-safe for concurrent execution
/// when they have no dependencies between them.
class DART_API ComputeNode
{
public:
  virtual ~ComputeNode() = default;

  virtual void execute(const ExecutionContext& ctx) = 0;

  [[nodiscard]] NodeId getId() const
  {
    return mId;
  }

  [[nodiscard]] const std::string& getName() const
  {
    return mName;
  }

  void setName(std::string_view name)
  {
    mName = std::string(name);
  }

  [[nodiscard]] virtual bool isParallelSafe() const
  {
    return true;
  }

protected:
  ComputeNode() = default;

  explicit ComputeNode(std::string_view name) : mName(name) {}

private:
  friend class ComputeGraph;
  NodeId mId{kInvalidNodeId};
  std::string mName;
};

/// Lambda-based node for simple computations
class DART_API LambdaNode : public ComputeNode
{
public:
  using Callable = std::function<void(const ExecutionContext&)>;

  explicit LambdaNode(Callable func, std::string_view name = "lambda")
    : ComputeNode(name), mFunc(std::move(func))
  {
  }

  void execute(const ExecutionContext& ctx) override
  {
    mFunc(ctx);
  }

private:
  Callable mFunc;
};

} // namespace dart::simulation
