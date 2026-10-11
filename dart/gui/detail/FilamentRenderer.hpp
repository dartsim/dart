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

#include <dart/gui/detail/debug.hpp>
#include <dart/gui/detail/renderable.hpp>

#include <Eigen/Core>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <cstdint>
struct ImDrawData;

namespace dart::gui::detail {
// Filament stays behind this C++17 boundary; only the implementation uses its
// SDK.
class DART_GUI_API FilamentRenderer
{
public:
  FilamentRenderer(
      int width, int height, bool headless, void* nativeWindow = nullptr);
  ~FilamentRenderer();
  FilamentRenderer(const FilamentRenderer&) = delete;
  FilamentRenderer& operator=(const FilamentRenderer&) = delete;
  void synchronize(const std::vector<RenderableDescriptor>& descriptors);
  void setCamera(
      const Eigen::Vector3d& eye,
      const Eigen::Vector3d& center,
      const Eigen::Vector3d& up,
      double fovyDegrees,
      double nearPlane,
      double farPlane);
  void resize(int width, int height);
  // GL clip z/w coefficients from the last completed frame.
  std::optional<Eigen::Vector4d> getLastDepthProjection() const;
  bool frame(bool captureRGBA = false);
  // Consume a readback queued by a successful frame(true), without rendering
  // another frame. Both capture APIs return tightly packed top-left RGBA.
  std::vector<std::uint8_t> consumeCapturedPixelsRGBA();
  std::vector<std::uint8_t> readPixelsRGBA();
  void renderImGui(const ImDrawData* drawData);
  void setShadowsEnabled(bool enabled);
  void setShadowsEnabled(RenderableId id, bool casts, bool receives);
  void setHeadlightsEnabled(bool enabled);
  void setLightingUp(const Eigen::Vector3d& up);
  void setBackgroundColor(const Eigen::Vector4d& rgba);
  void setDepthMode(bool enabled);
  void setWireframe(bool enabled);
  void setWireframe(RenderableId id, bool enabled);
  void setDebugScene(const DebugScene& debug);
  const std::string& getBackendName() const;
  const std::string& getDeviceName() const;
  // Actual SDK resource count for lifetime qualification of this renderer.
  std::size_t getTextureCount() const;

private:
  class Impl;
  std::unique_ptr<Impl> mImpl;
};
} // namespace dart::gui::detail
