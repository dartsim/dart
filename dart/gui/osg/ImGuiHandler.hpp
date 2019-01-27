/*
 * Copyright (c) 2011-2019, The DART development contributors
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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#ifndef DART_GUI_OSG_IMGUIHANDLER_HPP_
#define DART_GUI_OSG_IMGUIHANDLER_HPP_

#include <array>
#include <memory>
#include <vector>

#include <osg/GraphicsContext>
#include <osgGA/GUIActionAdapter>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIEventHandler>

namespace dart {
namespace gui {
namespace osg {

class ImGuiWidget;

class ImGuiHandler : public osgGA::GUIEventHandler
{
public:
  /// Constructor
  ImGuiHandler();

  void init();

  void newFrame(::osg::RenderInfo& renderInfo);

  void render(::osg::RenderInfo& renderInfo);

  void setCameraCallbacks(::osg::Camera* camera);

  //----------------------------------------------------------------------------
  /// \{ \name Widget management
  //----------------------------------------------------------------------------

  /// Returns true if this Viewer contains given widget.
  bool hasWidget(const std::shared_ptr<ImGuiWidget>& widget) const;

  /// Adds given Widget to this Viewer.
  void addWidget(const std::shared_ptr<ImGuiWidget>& widget,
                 bool visible = true);

  /// Removes given Widget from this Viewer.
  void removeWidget(const std::shared_ptr<ImGuiWidget>& widget);

  /// Removes all the widgets in this Viewer.
  void removeAllWidget();

  /// \}

  // Documentation inherited.
  bool handle(const osgGA::GUIEventAdapter& eventAdapter,
              osgGA::GUIActionAdapter& actionAdapter,
              ::osg::Object* object,
              ::osg::NodeVisitor* nodeVisitor) override;

protected:
  double mTime;

  /// Mouse buttons: left, right, middle.
  std::array<bool, 3> mMousePressed;

  float mMouseWheel;

  GLuint mFontTexture;

  std::vector<std::shared_ptr<ImGuiWidget>> mWidgets;
};

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_IMGUIHANDLER_HPP_
