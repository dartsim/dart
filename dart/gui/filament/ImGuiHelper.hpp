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
 * Copyright (C) 2016 The Android Open Source Project
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

#ifndef DART_GUI_FILAMENT_IMGUIHELPER_HPP_
#define DART_GUI_FILAMENT_IMGUIHELPER_HPP_

#include <vector>

#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <utils/Entity.h>

#include "dart/external/imgui/imgui.h"
#include "Path.hpp"

namespace dart {
namespace gui {
namespace flmt {

namespace filagui {

// Translates ImGui's draw commands into Filament primitives, textures, vertex
// buffers, etc.
// Creates a UI-specific Scene object and populates it with a Renderable. Does
// not handle
// event processing; clients can simply call ImGui::GetIO() directly and set the
// mouse state.
class ImGuiHelper
{
public:
  // Using std::function instead of a vanilla C callback to make it easy for
  // clients to pass in
  // lambdas that have captures.
  using Callback = std::function<void(filament::Engine*, filament::View*)>;

  // The constructor creates its own Scene and places it in the given View.
  ImGuiHelper(
      filament::Engine* engine,
      filament::View* view,
      const utils::Path& fontPath);
  ~ImGuiHelper();

  // Informs ImGui of the current display size, as well as the pixel ratio for
  // high DPI displays.
  void setDisplaySize(
      int width, int height, float scaleX = 0.0f, float scaleY = 0.0f);

  // This does not actually "render" in the sense of issuing OpenGL commands,
  // instead it populates the Filament View. Clients are responsible for
  // rendering the View. This should be called on every frame, regardless of
  // whether the Renderer wants to skip or not.
  void render(float timeStepInSeconds, Callback imguiCommands);

private:
  void renderDrawData(ImDrawData* imguiData);
  void createBuffers(int numRequiredBuffers);
  void populateVertexData(
      size_t bufferIndex,
      size_t vbSizeInBytes,
      void* vbData,
      size_t ibSizeInBytes,
      void* ibData);
  void createVertexBuffer(size_t bufferIndex, size_t capacity);
  void createIndexBuffer(size_t bufferIndex, size_t capacity);
  void syncThreads();
  filament::Engine* mEngine;
  filament::View* mView;
  filament::Material const* mMaterial = nullptr;
  std::vector<filament::VertexBuffer*> mVertexBuffers;
  std::vector<filament::IndexBuffer*> mIndexBuffers;
  std::vector<filament::MaterialInstance*> mMaterialInstances;
  ::utils::Entity mRenderable;
  filament::Texture* mTexture = nullptr;
  bool mHasSynced = false;
};

} // namespace filagui

} // namespace flmt
} // namespace gui
} // namespace dart

#endif // DART_GUI_FILAMENT_IMGUIHELPER_HPP_
