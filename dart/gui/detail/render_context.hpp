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

#include <dart/gui/Export.hpp>
#include <dart/gui/detail/backend_fwd.hpp>
#include <dart/gui/detail/headless_vulkan_platform.hpp>

#include <utils/Entity.h>

#include <memory>
#include <string>

namespace filament::backend {
class Platform;
}
namespace dart::gui::detail {
struct FilamentRenderContext
{
  ::filament::Engine* engine = nullptr;
  ::filament::Renderer* renderer = nullptr;
  ::filament::SwapChain* swapChain = nullptr;
  ::filament::View* view = nullptr;
  ::filament::Scene* scene = nullptr;
  ::filament::Camera* camera = nullptr;
  utils::Entity cameraEntity;
  std::unique_ptr<::filament::backend::Platform, void (*)(void*)> platform{
      nullptr, dartDestroyFilamentPlatform};
  std::string backendName;
  std::string deviceName;
};
DART_GUI_API FilamentRenderContext createFilamentRenderContext(
    int width, int height, bool headless, void* nativeWindow);
DART_GUI_API void checkFilamentBackendAvailability(bool headless);
DART_GUI_API void destroyFilamentRenderContext(FilamentRenderContext& context);
} // namespace dart::gui::detail
