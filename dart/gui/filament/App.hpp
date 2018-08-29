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

#ifndef DART_GUI_FILAMENT_APP_HPP_
#define DART_GUI_FILAMENT_APP_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <SDL2/SDL.h>

#include <filament/Engine.h>
#include <filament/Viewport.h>

#include "dart/gui/filament/CameraManipulator.hpp"
#include "dart/gui/filament/Config.hpp"
#include "dart/gui/filament/IBL.hpp"
#include "dart/gui/filament/ImGuiHelper.hpp"
#include "dart/gui/filament/Path.hpp"

namespace filament {
class Renderer;
class Scene;
class View;
} // namespace filament

class IBL;
class MeshAssimp;

namespace dart {
namespace gui {
namespace flmt {

class FilamentApp
{
public:
  using SetupCallback = std::function<void(
      filament::Engine*, filament::View*, filament::Scene*)>;
  using CleanupCallback = std::function<void(
      filament::Engine*, filament::View*, filament::Scene*)>;
  using PreRenderCallback = std::function<void(
      filament::Engine*,
      filament::View*,
      filament::Scene*,
      filament::Renderer*)>;
  using PostRenderCallback = std::function<void(
      filament::Engine*,
      filament::View*,
      filament::Scene*,
      filament::Renderer*)>;
  using ImGuiCallback = std::function<void(filament::Engine*, filament::View*)>;
  using AnimCallback
      = std::function<void(filament::Engine*, filament::View*, double now)>;

  static FilamentApp& get();

  ~FilamentApp();

  void setEngine(filament::Engine* engine)
  {
    mEngine = engine;
  }

  void animate(AnimCallback animation)
  {
    mAnimation = animation;
  }

  void run(
      const Config& config,
      SetupCallback setup,
      CleanupCallback cleanup,
      ImGuiCallback imgui = ImGuiCallback(),
      PreRenderCallback preRender = PreRenderCallback(),
      PostRenderCallback postRender = PostRenderCallback(),
      size_t width = 1024,
      size_t height = 640);

  filament::Material const* getDefaultMaterial() const noexcept
  {
    return mDefaultMaterial;
  }
  filament::Material const* getTransparentMaterial() const noexcept
  {
    return mTransparentMaterial;
  }
  IBL* getIBL() const noexcept
  {
    return mIBL.get();
  }

  void close()
  {
    mClosed = true;
  }

  FilamentApp(const FilamentApp& rhs) = delete;
  FilamentApp(FilamentApp&& rhs) = delete;
  FilamentApp& operator=(const FilamentApp& rhs) = delete;
  FilamentApp& operator=(FilamentApp&& rhs) = delete;

  // Returns the path to the Filament root for loading assets. This is
  // determined from the
  // executable folder, which allows users to launch samples from any folder.
  static const utils::Path& getRootPath()
  {
    static const utils::Path root
        = utils::Path::getCurrentExecutable().getParent();
    return root;
  }

private:
  FilamentApp();

  friend class Window;
  void initSDL();

  void loadIBL(const Config& config);

  filament::Engine* mEngine = nullptr;
  filament::Scene* mScene = nullptr;
  std::unique_ptr<IBL> mIBL;
  bool mClosed = false;
  uint64_t mTime = 0;

  filament::Material const* mDefaultMaterial = nullptr;
  filament::Material const* mTransparentMaterial = nullptr;
  filament::Material const* mDepthMaterial = nullptr;
  filament::MaterialInstance* mDepthMI = nullptr;
  std::unique_ptr<filagui::ImGuiHelper> mImGuiHelper;
  AnimCallback mAnimation;
};

} // namespace flmt
} // namespace gui
} // namespace dart

#endif // DART_GUI_FILAMENT_APP_HPP_
