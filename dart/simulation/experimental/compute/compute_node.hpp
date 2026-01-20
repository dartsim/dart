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

#include <dart/simulation/experimental/export.hpp>

#include <functional>
#include <string>
#include <string_view>

namespace dart::simulation::experimental::compute {

/// @brief A node in the compute graph representing a unit of work.
///
/// ComputeNode encapsulates a callable work function with a unique name.
/// Nodes are the fundamental building blocks of the compute graph and
/// are executed by ComputeExecutor implementations.
///
/// @note ComputeNode is non-copyable but movable. Once added to a ComputeGraph,
/// the node should not be moved.
class DART_EXPERIMENTAL_API ComputeNode
{
public:
  /// Function type for the work to be executed
  using ExecuteFn = std::function<void()>;

  /// @brief Constructs a compute node with the given name and work function.
  /// @param name Unique name for debugging and visualization
  /// @param fn The work function to execute
  ComputeNode(std::string name, ExecuteFn fn);

  /// Default destructor
  ~ComputeNode() = default;

  /// Non-copyable
  ComputeNode(const ComputeNode&) = delete;
  ComputeNode& operator=(const ComputeNode&) = delete;

  /// Movable
  ComputeNode(ComputeNode&&) noexcept = default;
  ComputeNode& operator=(ComputeNode&&) noexcept = default;

  /// @brief Gets the name of this node.
  /// @return The node name
  [[nodiscard]] const std::string& getName() const noexcept
  {
    return m_name;
  }

  /// @brief Executes the work function.
  void execute();

  /// @brief Checks if this node has a valid work function.
  /// @return True if the work function is callable
  [[nodiscard]] bool isValid() const noexcept
  {
    return static_cast<bool>(m_fn);
  }

private:
  std::string m_name;
  ExecuteFn m_fn;
};

} // namespace dart::simulation::experimental::compute
