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

#ifndef DART_GUI_VIEWER_BACKEND_HPP_
#define DART_GUI_VIEWER_BACKEND_HPP_

#include <dart/gui/export.hpp>
#include <dart/gui/input_event.hpp>
#include <dart/gui/scene.hpp>

#include <Eigen/Core>

#include <optional>
#include <string>
#include <vector>

namespace dart {
namespace gui {

struct DART_GUI_API ViewerConfig
{
  int width = 1280;
  int height = 720;
  std::string title = "DART Viewer";
  int target_fps = 60;
  bool headless = false;
  Eigen::Vector3d upwards_direction{0.0, 0.0, 1.0};
};

class DART_GUI_API ViewerBackend
{
public:
  virtual ~ViewerBackend() = default;
  virtual bool initialize(const ViewerConfig& config) = 0;
  virtual bool shouldClose() const = 0;
  virtual void beginFrame() = 0;
  virtual void render(const Scene& scene) = 0;
  virtual void endFrame() = 0;
  virtual void shutdown() = 0;
  virtual std::vector<InputEvent> pollEvents() = 0;

  /// Ray-cast pick: returns the nearest hit among scene nodes, if any.
  /// @param scene The current scene snapshot
  /// @param screen_x Mouse X in screen coordinates
  /// @param screen_y Mouse Y in screen coordinates
  virtual std::optional<HitResult> pickNode(
      const Scene& scene, float screen_x, float screen_y)
      = 0;

  /// Queue a screenshot to be captured during the next render() call.
  virtual void captureScreenshot(const std::string& filename) = 0;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_VIEWER_BACKEND_HPP_
