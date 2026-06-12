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

#include "dart/simulation/compute/compute_graph_visualization.hpp"

#include "dart/simulation/compute/compute_graph.hpp"
#include "dart/simulation/compute/compute_node.hpp"
#include "dart/simulation/compute/compute_stage_metadata.hpp"

#include <chrono>
#include <sstream>
#include <string_view>

namespace dart::simulation::compute {

namespace {

//==============================================================================
std::string escapeDot(std::string_view text)
{
  std::string escaped;
  escaped.reserve(text.size());
  for (const auto c : text) {
    switch (c) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      default:
        escaped += c;
        break;
    }
  }
  return escaped;
}

//==============================================================================
long long toMicroseconds(ComputeExecutionProfile::Duration duration)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(duration)
      .count();
}

//==============================================================================
std::string nodeId(const ComputeNode& node)
{
  return "node_" + escapeDot(node.getName());
}

//==============================================================================
void appendNodeLabel(
    std::ostringstream& out,
    const ComputeNode& node,
    const ComputeExecutionProfile* profile,
    const ComputeGraphDotOptions& options)
{
  out << escapeDot(node.getName());

  if (options.includeMetadata) {
    const auto& metadata = node.getMetadata();
    out << "\\ndomain=" << toString(metadata.domain);
    out << "\\naccel="
        << escapeDot(formatAccelerationMask(metadata.acceleration));
  }

  if (options.includeResources) {
    for (const auto& access : node.getMetadata().resources) {
      out << "\\n"
          << escapeDot(std::string(toString(access.mode))) << " "
          << escapeDot(
                 std::string_view{
                     access.resource.data(), access.resource.size()});
    }
  }

  if (options.includeProfile && profile) {
    if (const auto* nodeProfile = profile->getNode(node.getName())) {
      out << "\\nlevel=" << nodeProfile->level;
      out << "\\nworker=" << nodeProfile->workerIndex;
      out << "\\nduration_us=" << toMicroseconds(nodeProfile->duration);
    }
  }
}

} // namespace

//==============================================================================
std::string toDot(
    const ComputeGraph& graph,
    const ComputeExecutionProfile* profile,
    const ComputeGraphDotOptions& options)
{
  std::ostringstream out;
  out << "digraph ComputeGraph {\n";
  out << "  rankdir=LR;\n";
  out << "  node [shape=box, style=rounded];\n";

  const auto nodes = graph.getNodes();
  for (const auto* node : nodes) {
    out << "  \"" << nodeId(*node) << "\" [label=\"";
    appendNodeLabel(out, *node, profile, options);
    out << "\"];\n";
  }

  for (const auto& edge : graph.getEdges()) {
    out << "  \"" << nodeId(*edge.from) << "\" -> \"" << nodeId(*edge.to)
        << "\";\n";
  }

  if (options.groupParallelLevels) {
    const auto levels = graph.getParallelLevels();
    for (std::size_t i = 0; i < levels.size(); ++i) {
      out << "  { rank=same;";
      for (const auto* node : levels[i]) {
        out << " \"" << nodeId(*node) << "\";";
      }
      out << " }\n";
    }
  }

  out << "}\n";
  return out.str();
}

} // namespace dart::simulation::compute
