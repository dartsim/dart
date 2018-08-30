/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

/*
 * Copyright (C) 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <dart/dart.hpp>
#include <dart/gui/filament/filament.hpp>
#include <dart/external/imgui/imgui.h>

using namespace dart;

int main()
{
  bool showDemo = false;
  bool showMetrics = false;
  auto imgui = [&showDemo, &showMetrics] (filament::Engine*, filament::View*) {
      // In ImGui, the window title is a unique identifier, so don't call this "ImGui Demo" to
      // avoid colliding with ImGui::ShowDemoWindow().
      ImGui::Begin("ImGui", nullptr, ImGuiWindowFlags_MenuBar);
      if (ImGui::BeginMenuBar()) {
          if (ImGui::BeginMenu("File")) {
              if (ImGui::MenuItem("Close"))  {
                  gui::flmt::FilamentApp::get().close();
              }
              ImGui::EndMenu();
          }
          ImGui::EndMenuBar();
      }
      ImGui::Checkbox("Widgets", &showDemo);
      ImGui::Checkbox("Metrics", &showMetrics);
      if (showDemo) {
          ImGui::ShowDemoWindow(&showDemo);
      }
      if (showMetrics) {
          ImGui::ShowMetricsWindow(&showMetrics);
      }
      ImGui::End();
  };

  gui::flmt::Config config;
  config.backend = filament::Engine::Backend::OPENGL;
  config.title = "ImGui Demo";
  auto nop = [](filament::Engine*, filament::View*, filament::Scene*) {};
  gui::flmt::FilamentApp::get().run(config, nop, nop, imgui);

  return 0;
}
