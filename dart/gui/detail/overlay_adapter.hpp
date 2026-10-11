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

#include <dart/gui/ViewerAttachment.hpp>
#include <dart/gui/detail/debug.hpp>

#include <memory>
#include <vector>

namespace dart::gui {
class DebugOverlay;
class GridVisual;
class SupportPolygonVisual;
namespace detail {
struct OverlayAccess
{
  static void append(const DebugOverlay&, DebugScene&);
  static void append(const GridVisual&, DebugScene&);
  static void append(const SupportPolygonVisual&, DebugScene&);
};

/// Replaces scene contents with annotations from the current attachments.
DART_GUI_API void collectDebugScene(
    const std::vector<std::shared_ptr<ViewerAttachment>>& attachments,
    DebugScene& scene);

/// Expand depth-tested pixel-width lines into camera-facing triangles.
/// Always-on-top pixel-width lines stay available to the ImGui renderer.
DART_GUI_API void prepareDebugScene(
    DebugScene& scene,
    const Eigen::Vector3d& eye,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& up,
    double fovyDegrees,
    int width,
    int height);

class DART_GUI_API OverlayRenderer
{
public:
  OverlayRenderer();
  ~OverlayRenderer();
  OverlayRenderer(const OverlayRenderer&) = delete;
  OverlayRenderer& operator=(const OverlayRenderer&) = delete;
  /// Call with this viewer's ImGui context current, before ImGui::NewFrame().
  void prepareFonts(const DebugScene& scene);
  /// Call after NewFrame(), before ImGui::Render(). Dimensions are physical
  /// pixels.
  void render(
      const DebugScene& scene,
      const Eigen::Vector3d& eye,
      const Eigen::Vector3d& center,
      const Eigen::Vector3d& up,
      double fovyDegrees,
      int width,
      int height);

private:
  struct Impl;
  std::unique_ptr<Impl> mImpl;
};
} // namespace detail
} // namespace dart::gui
