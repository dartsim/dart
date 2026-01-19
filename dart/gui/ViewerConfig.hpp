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

#ifndef DART_GUI_VIEWERCONFIG_HPP_
#define DART_GUI_VIEWERCONFIG_HPP_

#include <dart/gui/export.hpp>

#include <osg/Vec4>

namespace dart {
namespace gui {

/// Rendering mode for the viewer
enum class RenderingMode
{
  Window,
  Headless,
};

/// Configuration bundle used when constructing a Viewer.
struct DART_GUI_API ViewerConfig final
{
  RenderingMode mode = RenderingMode::Window;

  int width = 1024;

  int height = 768;

  /// MSAA samples (0 = disabled, recommended for determinism in headless mode)
  int msaaSamples = 0;

  /// Use software renderer (OSMesa) for deterministic output.
  /// Only applies when mode == Headless. Requires OSMesa at build time.
  bool useSoftwareRenderer = false;

  ::osg::Vec4 clearColor = ::osg::Vec4(0.9f, 0.9f, 0.9f, 1.0f);

  ViewerConfig() = default;

  static ViewerConfig headless(int w = 1024, int h = 768)
  {
    ViewerConfig config;
    config.mode = RenderingMode::Headless;
    config.width = w;
    config.height = h;
    return config;
  }
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_VIEWERCONFIG_HPP_
