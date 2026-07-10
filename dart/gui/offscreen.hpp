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

#ifndef DART_GUI_OFFSCREEN_HPP_
#define DART_GUI_OFFSCREEN_HPP_

#include <dart/gui/debug.hpp>
#include <dart/gui/export.hpp>
#include <dart/gui/renderable.hpp>

#include <memory>
#include <string>
#include <vector>

#include <cstdint>

namespace dart::gui {

struct OrbitCamera;

struct DART_GUI_API RenderedImage
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t channels = 4;
  std::vector<std::uint8_t> pixels;
};

struct DART_GUI_API OffscreenRenderOptions
{
  int width = 640;
  int height = 480;
  std::string renderBackend;
  int warmupFrames = 2;
  bool highFidelity = false;
};

class DART_GUI_API OffscreenRenderer
{
public:
  explicit OffscreenRenderer(const OffscreenRenderOptions& options = {});
  ~OffscreenRenderer();

  OffscreenRenderer(const OffscreenRenderer&) = delete;
  OffscreenRenderer& operator=(const OffscreenRenderer&) = delete;

  OffscreenRenderer(OffscreenRenderer&&) noexcept;
  OffscreenRenderer& operator=(OffscreenRenderer&&) noexcept;

  RenderedImage render(
      const std::vector<RenderableDescriptor>& descriptors,
      const OrbitCamera& camera);

  /// Renders the descriptors with an application-supplied debug overlay drawn
  /// through the same unlit, always-on-top path as the interactive viewer.
  /// Labels are not rasterized here (the viewer draws them in its UI pass);
  /// callers composite them onto the returned image instead.
  RenderedImage render(
      const std::vector<RenderableDescriptor>& descriptors,
      const OrbitCamera& camera,
      const DebugScene& debug);

private:
  struct Impl;
  std::unique_ptr<Impl> mImpl;
};

} // namespace dart::gui

#endif // DART_GUI_OFFSCREEN_HPP_
