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

#include <dart/simulation/experimental/compute/execution_profile.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <string>

namespace dart::simulation::experimental::compute {

class ComputeGraph;

/// Options for exporting a compute graph to Graphviz DOT.
struct DART_EXPERIMENTAL_API ComputeGraphDotOptions
{
  bool includeMetadata = true;
  bool includeProfile = true;
  bool groupParallelLevels = true;
  bool includeResources = true;
};

/// Export a compute graph as Graphviz DOT for debugging and visualization.
///
/// The DOT output is backend-neutral and can include static metadata plus an
/// optional execution profile. This is intentionally a text artifact instead of
/// a renderer dependency, so future GUI/rendering tools can consume the same
/// data without coupling the compute graph to a specific visualization stack.
[[nodiscard]] DART_EXPERIMENTAL_API std::string toDot(
    const ComputeGraph& graph,
    const ComputeExecutionProfile* profile = nullptr,
    const ComputeGraphDotOptions& options = {});

} // namespace dart::simulation::experimental::compute
