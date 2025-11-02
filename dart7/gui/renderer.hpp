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

#pragma once

#include <cstdint>
#include <string>

namespace dart7::gui {

/// Rendering backend selection. Headless rendering keeps GPU resources dormant
/// until a windowed renderer is requested.
enum class BackingMode
{
  Headless,
  Window
};

/// Options passed to the Renderer to control the initial output surface.
struct RendererOptions
{
  BackingMode mode{BackingMode::Headless};
  std::string windowTitle{"dart7"};
  std::uint32_t width{1280};
  std::uint32_t height{720};
};

/// Minimal placeholder renderer that will back the VulkanSceneGraph + ImGui
/// integration in future revisions. For now it simply records configuration and
/// exposes a few no-op hooks that higher layers can call without needing a
/// concrete backend.
class Renderer
{
public:
  Renderer();
  explicit Renderer(RendererOptions options);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;
  Renderer(Renderer&&) noexcept;
  Renderer& operator=(Renderer&&) noexcept;

  const RendererOptions& options() const;
  bool isHeadless() const;

  void renderFrame();
  void pollEvents();

private:
  RendererOptions mOptions;
};

} // namespace dart7::gui
