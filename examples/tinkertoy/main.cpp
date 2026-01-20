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

#include "tinkertoy_widget.hpp"
#include "tinkertoy_world_node.hpp"

#include <dart/config.hpp>

#include <dart/gui/all.hpp>
#include <dart/gui/im_gui_handler.hpp>

#include <dart/All.hpp>

#include <CLI/CLI.hpp>

//==============================================================================
class TinkertoyInputHandler : public osgGA::GUIEventHandler
{
public:
  TinkertoyInputHandler(dart::gui::Viewer* viewer, TinkertoyWorldNode* node)
    : mViewer(viewer), mNode(node)
  {
    // Do nothing
  }

  bool handle(
      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
      if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Tab) {
        mViewer->home();
        return true;
      } else if (ea.getKey() == '1') {
        mNode->addWeldJointBlock();
        return true;
      } else if (ea.getKey() == '2') {
        mNode->addRevoluteJointBlock();
        return true;
      } else if (ea.getKey() == '3') {
        mNode->addBallJointBlock();
        return true;
      } else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_BackSpace) {
        mNode->clearPick();
        return true;
      } else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Delete) {
        mNode->deletePick();
        return true;
      } else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Up) {
        mNode->incrementForceCoeff();
        return true;
      } else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Down) {
        mNode->decrementForceCoeff();
        return true;
      } else if (ea.getKey() == '`') {
        mNode->reorientTarget();
        return true;
      } else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Return) {
        if (!mViewer->isRecording())
          mViewer->record(dart::config::dataPath("screencap"));
        else
          mViewer->pauseRecording();
        return true;
      }
    }

    return false;
  }

  dart::gui::Viewer* mViewer;
  TinkertoyWorldNode* mNode;
};

//==============================================================================
class TinkertoyMouseHandler : public dart::gui::MouseEventHandler
{
public:
  TinkertoyMouseHandler(TinkertoyInputHandler* inputHandler)
    : mInputHandler(inputHandler)
  {
    // Do nothing
  }

  void update() override
  {
    dart::gui::Viewer* viewer = mInputHandler->mViewer;
    TinkertoyWorldNode* node = mInputHandler->mNode;

    dart::gui::MouseButtonEvent event
        = viewer->getDefaultEventHandler()->getButtonEvent(
            dart::gui::LEFT_MOUSE);

    if (dart::gui::BUTTON_PUSH == event) {
      const auto picks = viewer->getDefaultEventHandler()->getButtonPicks(
          dart::gui::LEFT_MOUSE, dart::gui::BUTTON_PUSH);

      if (picks.empty())
        return;

      node->handlePick(picks.front());
    }
  }

  TinkertoyInputHandler* mInputHandler;
};

//==============================================================================
int main(int argc, char* argv[])
{
  CLI::App app("Tinkertoy builder example");
  double guiScale = 1.0;
  app.add_option("--gui-scale", guiScale, "Scale factor for ImGui widgets")
      ->check(CLI::PositiveNumber);
  CLI11_PARSE(app, argc, argv);

  // Create a world
  dart::simulation::WorldPtr world(new dart::simulation::World);

  // Create some coordinate axes for the user's reference
  dart::gui::InteractiveFramePtr coordinates
      = dart::gui::InteractiveFrame::createShared(
          dart::dynamics::Frame::World(),
          "coordinates",
          Eigen::Isometry3d::Identity(),
          0.2);

  for (size_t i = 0; i < 3; ++i)
    for (size_t j = 0; j < 3; ++j)
      coordinates->getTool((dart::gui::InteractiveTool::Type)(i), j)
          ->setEnabled(false);

  world->addSimpleFrame(coordinates);

  // Create the OSG Node that represents the world
  osg::ref_ptr<TinkertoyWorldNode> node = new TinkertoyWorldNode(world);

  // Create the viewer
  osg::ref_ptr<dart::gui::ImGuiViewer> viewer = new dart::gui::ImGuiViewer();
  viewer->setImGuiScale(static_cast<float>(guiScale));
  viewer->addWorldNode(node);

  // Add control widget for atlas
  viewer->getImGuiHandler()->addWidget(
      std::make_shared<TinkertoyWidget>(viewer, node.get()));

  // Add the keyboard input handler
  osg::ref_ptr<TinkertoyInputHandler> input
      = new TinkertoyInputHandler(viewer, node);
  viewer->addEventHandler(input.get());

  // Add the mouse input handler
  std::shared_ptr<TinkertoyMouseHandler> mouse
      = std::make_shared<TinkertoyMouseHandler>(input);
  viewer->getDefaultEventHandler()->addMouseEventHandler(mouse.get());

  // Set the dimensions for the window
  viewer->setUpViewInWindow(0, 0, 1280, 720);

  // Set the window name
  viewer->realize();
  osgViewer::Viewer::Windows windows;
  viewer->getWindows(windows);
  windows.front()->setWindowName("Tinkertoy");

  // Run the GUI application
  viewer->run();
}
