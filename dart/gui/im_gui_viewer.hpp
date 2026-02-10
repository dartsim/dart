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

#ifndef DART_GUI_IMGUIVIEWER_HPP_
#define DART_GUI_IMGUIVIEWER_HPP_

#include <dart/gui/export.hpp>
#include <dart/gui/im_gui_handler.hpp>
#include <dart/gui/viewer.hpp>

#include <memory>

namespace dart {
namespace gui {

class MainMenuWidget;
class AboutWidget;

class DART_GUI_API ImGuiViewer : public Viewer
{
public:
  /// Constructor for dart::gui::Viewer. This will automatically create the
  /// default event handler.
  ImGuiViewer(
      const ::osg::Vec4& clearColor = ::osg::Vec4(0.9f, 0.9f, 0.9f, 1.0f));

  /// Constructor with configuration bundle. Supports headless mode.
  explicit ImGuiViewer(const ViewerConfig& config);

  /// Destructor.
  virtual ~ImGuiViewer();

  /// Get ImGui handler.
  ImGuiHandler* getImGuiHandler();

  /// Get cosnt ImGui handler.
  const ImGuiHandler* getImGuiHandler() const;

  /// Set the ImGui global scale factor (fonts + widget sizes).
  void setImGuiScale(float scale);

  /// Set up the viewer window, scaling width/height by the ImGui scale.
  void setUpViewInWindow(int x, int y, int width, int height);

  /// Set up the viewer window on a specific screen, scaling width/height by the
  /// ImGui scale.
  void setUpViewInWindow(int x, int y, int width, int height, int screenNum);

  /// Show About widget.
  void showAbout();

  /// Hide About widget.
  void hideAbout();

protected:
  /// ImGui handler.
  ::osg::ref_ptr<ImGuiHandler> mImGuiHandler;

  /// About widget.
  std::shared_ptr<AboutWidget> mAboutWidget;

  float mImGuiScale{1.f};
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_IMGUIVIEWER_HPP_
