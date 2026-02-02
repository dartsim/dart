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

#include "dart/gui/raylib/inspector_widget.hpp"

#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/gui/scene.hpp"
#include "dart/gui/scene_extractor.hpp"
#include "dart/gui/scene_viewer.hpp"

#if __has_include(<imgui.h>)
  #include <imgui.h>
#elif __has_include(<imgui/imgui.h>)
  #include <imgui/imgui.h>
#endif

namespace dart {
namespace gui {

InspectorWidget::InspectorWidget(SceneViewer* viewer) : viewer_(viewer) {}

void InspectorWidget::render()
{
  ImGui::SetNextWindowPos(ImVec2(990, 10), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(280, 350), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Inspector", &mIsVisible)) {
    ImGui::End();
    return;
  }

  auto selected_id = viewer_->selectedNodeId();
  if (!selected_id) {
    ImGui::Text("No selection");
    ImGui::End();
    return;
  }

  const auto& scene = viewer_->scene();
  const auto& entity_map = viewer_->entityMap();

  auto node_it = std::find_if(
      scene.nodes.begin(),
      scene.nodes.end(),
      [selected_id](const SceneNode& node) { return node.id == selected_id; });

  if (node_it != scene.nodes.end()) {
    ImGui::Text("Node ID: %llu", static_cast<unsigned long long>(*selected_id));

    const auto& pos = node_it->transform.translation();
    ImGui::Text("Position: (%.3f, %.3f, %.3f)", pos.x(), pos.y(), pos.z());

    const auto& color = node_it->material.color;
    ImGui::Text(
        "Color: (%.2f, %.2f, %.2f, %.2f)",
        color.x(),
        color.y(),
        color.z(),
        color.w());
  }

  auto entity_it = entity_map.find(*selected_id);
  if (entity_it != entity_map.end()) {
    const auto& entity_info = entity_it->second;

    if (entity_info.body_node) {
      ImGui::Separator();
      ImGui::Text("Body Node: %s", entity_info.body_node->getName().c_str());
      ImGui::Text("Mass: %.3f", entity_info.body_node->getMass());

      auto* parent_joint = entity_info.body_node->getParentJoint();
      if (parent_joint) {
        auto joint_type = parent_joint->getType();
        ImGui::Text(
            "Parent Joint: %.*s",
            static_cast<int>(joint_type.size()),
            joint_type.data());
        ImGui::Text("Joint Positions: %zu", parent_joint->getNumDofs());
      }
    }

    if (entity_info.simple_frame) {
      ImGui::Separator();
      ImGui::Text(
          "Simple Frame: %s", entity_info.simple_frame->getName().c_str());
    }
  }

  ImGui::End();
}

} // namespace gui
} // namespace dart
