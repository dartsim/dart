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

#include "dart/gui/raylib/sim_control_widget.hpp"

#include "dart/gui/scene_viewer.hpp"
#include "dart/simulation/world.hpp"

#if __has_include(<imgui.h>)
  #include <imgui.h>
#elif __has_include(<imgui/imgui.h>)
  #include <imgui/imgui.h>
#endif

namespace dart {
namespace gui {

SimControlWidget::SimControlWidget(SceneViewer* viewer) : viewer_(viewer)
{
  if (auto world = viewer_->primaryWorld()) {
    default_gravity_ = world->getGravity();
  }
}

void SimControlWidget::render()
{
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(280, 300), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Simulation Controls", &mIsVisible)) {
    ImGui::End();
    return;
  }

  if (ImGui::Button(viewer_->isPaused() ? "Play" : "Pause")) {
    if (viewer_->isPaused()) {
      viewer_->unpause();
    } else {
      viewer_->pause();
    }
  }
  ImGui::SameLine();
  bool paused = viewer_->isPaused();
  if (ImGui::Button("Step")) {
    viewer_->step();
  }
  if (!paused) {
    ImGui::BeginDisabled();
  }
  if (!paused) {
    ImGui::EndDisabled();
  }

  ImGui::Separator();

  auto world = viewer_->primaryWorld();
  if (world) {
    ImGui::Text("Time: %.3f", world->getTime());
  }
  ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
  ImGui::Text("RTF: %.2f", viewer_->getLastRealTimeFactor());

  ImGui::Separator();

  if (world) {
    float timestep = static_cast<float>(world->getTimeStep());
    if (ImGui::SliderFloat("dt", &timestep, 0.0001f, 0.01f, "%.4f")) {
      world->setTimeStep(static_cast<double>(timestep));
    }
  }

  int spc = static_cast<int>(viewer_->numStepsPerCycle());
  if (ImGui::SliderInt("Steps/cycle", &spc, 1, 100)) {
    viewer_->setNumStepsPerCycle(static_cast<std::size_t>(spc));
  }

  if (world) {
    if (ImGui::Checkbox("Gravity", &gravity_enabled_)) {
      if (gravity_enabled_) {
        world->setGravity(default_gravity_);
      } else {
        world->setGravity(Eigen::Vector3d::Zero());
      }
    }
  }

  ImGui::End();
}

} // namespace gui
} // namespace dart
