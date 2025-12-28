/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/gui/ImGuiHandler.hpp>
#include <dart/gui/ImGuiViewer.hpp>
#include <dart/gui/ImGuiWidget.hpp>
#include <dart/gui/IncludeImGui.hpp>

#include <dart/simulation/World.hpp>

#include <CLI/CLI.hpp>
#include <osg/Group>

#include <memory>

namespace {

class LcpDashboardWidget : public dart::gui::ImGuiWidget
{
public:
  explicit LcpDashboardWidget(dart::gui::ImGuiViewer* viewer) : mViewer(viewer)
  {
  }

  void render() override
  {
    ImGui::SetNextWindowPos(ImVec2(10, 20), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(520, 520), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.6f);

    if (!ImGui::Begin(
            "LCP Solver Dashboard",
            nullptr,
            ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoCollapse)) {
      ImGui::End();
      return;
    }

    if (ImGui::BeginMenuBar()) {
      if (ImGui::BeginMenu("Menu")) {
        if (ImGui::MenuItem("Exit"))
          mViewer->setDone(true);
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("About DART"))
          mViewer->showAbout();
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }

    ImGui::TextWrapped(
        "Placeholder UI. This example will showcase LCP solver behavior, "
        "performance, and tradeoffs.");

    ImGui::Separator();
    ImGui::Text("Next steps:");
    ImGui::BulletText("Scenario selector with references");
    ImGui::BulletText("Solver selector by category");
    ImGui::BulletText("Results, contract checks, and performance plots");

    ImGui::End();
  }

private:
  dart::gui::ImGuiViewer* mViewer;
};

} // namespace

int main(int argc, char* argv[])
{
  CLI::App app("LCP solver dashboard");
  double guiScale = 1.0;
  app.add_option("--gui-scale", guiScale, "Scale factor for ImGui widgets")
      ->check(CLI::PositiveNumber);
  CLI11_PARSE(app, argc, argv);

  auto world = dart::simulation::World::create("lcp_dashboard_world");
  auto worldNode = std::make_shared<dart::gui::WorldNode>(world);

  osg::ref_ptr<dart::gui::ImGuiViewer> viewer = new dart::gui::ImGuiViewer();
  viewer->setImGuiScale(static_cast<float>(guiScale));
  viewer->addWorldNode(worldNode);
  viewer->getImGuiHandler()->addWidget(
      std::make_shared<LcpDashboardWidget>(viewer));

  viewer->setUpViewInWindow(100, 100, 1280, 720);
  viewer->run();

  return 0;
}
