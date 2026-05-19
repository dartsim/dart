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

#include <dart/simulation/experimental/compute/compute_stage_metadata.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <functional>
#include <string>
#include <string_view>

namespace dart::simulation::experimental::compute {

/// A named unit of work in an experimental computation graph.
class DART_EXPERIMENTAL_API ComputeNode
{
public:
  using ExecuteFn = std::function<void()>;

  ComputeNode(
      std::string_view name, ExecuteFn fn, ComputeStageMetadata metadata = {});
  ~ComputeNode() = default;

  ComputeNode(const ComputeNode&) = delete;
  ComputeNode& operator=(const ComputeNode&) = delete;
  ComputeNode(ComputeNode&&) noexcept = default;
  ComputeNode& operator=(ComputeNode&&) noexcept = default;

  [[nodiscard]] const std::string& getName() const noexcept
  {
    return m_name;
  }

  [[nodiscard]] bool isValid() const noexcept
  {
    return static_cast<bool>(m_fn);
  }

  [[nodiscard]] const ComputeStageMetadata& getMetadata() const noexcept
  {
    return m_metadata;
  }

  void execute();

private:
  std::string m_name;
  ExecuteFn m_fn;
  ComputeStageMetadata m_metadata;
};

} // namespace dart::simulation::experimental::compute
