/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <memory>
#include <string>

#include "dart/gui/export.hpp"
#include "dart/rendering/osg_include.hpp"

namespace dart::gui {

// class DART_GUI_API ImGuiHandler : public osgGA::GUIEventHandler
//{
// public:
//  ImGuiHandler();

//  virtual ~ImGuiHandler();

//  void newFrame(::osg::RenderInfo& renderInfo);

//  void render(::osg::RenderInfo& renderInfo);

//  void setCameraCallbacks(::osg::Camera* camera);

//  //----------------------------------------------------------------------------
//  /// \{ \name Widget management
//  //----------------------------------------------------------------------------

//  /// Returns true if this Viewer contains given widget.
//  bool hasWidget(const std::shared_ptr<ImGuiWidget>& widget) const;

//  /// Adds given Widget to this Viewer.
//  void addWidget(
//      const std::shared_ptr<ImGuiWidget>& widget, bool visible = true);

//  /// Removes given Widget from this Viewer.
//  void removeWidget(const std::shared_ptr<ImGuiWidget>& widget);

//  /// Removes all the widgets in this Viewer.
//  void removeAllWidget();

//  /// \}

//  // Documentation inherited.
//  bool handle(
//      const osgGA::GUIEventAdapter& eventAdapter,
//      osgGA::GUIActionAdapter& actionAdapter,
//      ::osg::Object* object,
//      ::osg::NodeVisitor* nodeVisitor) override;
// private:
//  double mTime;

//  /// Mouse buttons: left, right, middle.
//  std::array<bool, 3> mMousePressed;

//  float mMouseWheel;

//  std::vector<std::shared_ptr<ImGuiWidget>> mWidgets;
//};

} // namespace dart::gui
