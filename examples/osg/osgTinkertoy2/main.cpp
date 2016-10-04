/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include "TinkertoyWorldNode.hpp"
#include "TinkertoyWidget.hpp"

//==============================================================================
class TinkertoyInputHandler : public osgGA::GUIEventHandler
{
public:

  TinkertoyInputHandler(dart::gui::osg::Viewer* viewer,
                        TinkertoyWorldNode* node)
    : mViewer(viewer),
      mNode(node),
      mGravityOn(true)
  {
    // Do nothing
  }

  bool handle(const osgGA::GUIEventAdapter& ea,
              osgGA::GUIActionAdapter&) override
  {
    if(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
    {
      if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Tab)
      {
        mViewer->home();
        return true;
      }
      else if(ea.getKey() == '1')
      {
        mNode->addWeldJointBlock();
        return true;
      }
      else if(ea.getKey() == '2')
      {
        mNode->addRevoluteJointBlock();
        return true;
      }
      else if(ea.getKey() == '3')
      {
        mNode->addBallJointBlock();
        return true;
      }
      else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_BackSpace)
      {
        mNode->clearPick();
        return true;
      }
      else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Delete)
      {
        mNode->deletePick();
        return true;
      }
      else if(ea.getKey() == 'g' || ea.getKey() == 'G')
      {
        mGravityOn = !mGravityOn;

        if(mGravityOn)
        {
          mNode->getWorld()->setGravity(-9.81*Eigen::Vector3d::UnitZ());
          std::cout << "[Gravity: On]" << std::endl;
        }
        else
        {
          mNode->getWorld()->setGravity(Eigen::Vector3d::Zero());
          std::cout << "[Gravity: Off]" << std::endl;
        }

        return true;
      }
      else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Up)
      {
        mNode->incrementForceCoeff();
        return true;
      }
      else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Down)
      {
        mNode->decrementForceCoeff();
        return true;
      }
      else if(ea.getKey() == '`')
      {
        mNode->reorientTarget();
        return true;
      }
    }

    return false;
  }


  dart::gui::osg::Viewer* mViewer;
  TinkertoyWorldNode* mNode;

  bool mGravityOn;
};

//==============================================================================
class TinkertoyMouseHandler : public dart::gui::osg::MouseEventHandler
{
public:

  TinkertoyMouseHandler(TinkertoyInputHandler* inputHandler)
    : mInputHandler(inputHandler)
  {
    // Do nothing
  }

  void update() override
  {
    dart::gui::osg::Viewer* viewer = mInputHandler->mViewer;
    TinkertoyWorldNode* node = mInputHandler->mNode;

    dart::gui::osg::MouseButtonEvent event =
        viewer->getDefaultEventHandler()->
        getButtonEvent(dart::gui::osg::LEFT_MOUSE);

    if(dart::gui::osg::BUTTON_PUSH == event)
    {
      const std::vector<dart::gui::osg::PickInfo>& picks =
          viewer->getDefaultEventHandler()->getButtonPicks(
            dart::gui::osg::LEFT_MOUSE, dart::gui::osg::BUTTON_PUSH);

      if(picks.empty())
        return;

      node->handlePick(picks.front());
    }
  }

  TinkertoyInputHandler* mInputHandler;
};

//==============================================================================
int main()
{
  // Create a world
  dart::simulation::WorldPtr world(new dart::simulation::World);

  // Create some coordinate axes for the user's reference
  dart::gui::osg::InteractiveFramePtr coordinates =
      std::make_shared<dart::gui::osg::InteractiveFrame>(
        dart::dynamics::Frame::World(), "coordinates",
        Eigen::Isometry3d::Identity(), 0.2);

  for(size_t i=0; i < 3; ++i)
    for(size_t j=0; j < 3; ++j)
      coordinates->getTool((dart::gui::osg::InteractiveTool::Type)(i), j)->
          setEnabled(false);

  world->addSimpleFrame(coordinates);

  // Create the OSG Node that represents the world
  osg::ref_ptr<TinkertoyWorldNode> node = new TinkertoyWorldNode(world);
  node->setNumStepsPerCycle(20);

  // Create the viewer
  dart::gui::osg::ImGuiViewer viewer;
  viewer.addWorldNode(node);

  // Add control widget for atlas
  viewer.getImGuiHandler()->addWidget(
        std::make_shared<TinkertoyWidget>(node.get()));

  // Add the keyboard input handler
  osg::ref_ptr<TinkertoyInputHandler> input =
      new TinkertoyInputHandler(&viewer, node);
  viewer.addEventHandler(input.get());

  // Add the mouse input handler
  std::shared_ptr<TinkertoyMouseHandler> mouse =
      std::make_shared<TinkertoyMouseHandler>(input);
  viewer.getDefaultEventHandler()->addMouseEventHandler(mouse.get());

  // Set the dimensions for the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set the window name
  viewer.realize();
  osgViewer::Viewer::Windows windows;
  viewer.getWindows(windows);
  windows.front()->setWindowName("Tinkertoy");

  // Print out instructions for the user
  std::cout << viewer.getInstructions() << std::endl;

  std::cout << "Left-click on a block to select it.\n"
            << "Press [Backspace] to deselect.\n"
            << "Press [Tab] to reset the camera view\n"
            << "Press [`] to reset the orientation of the target\n"
            << "\n --- While Simulation is Paused ---\n"
            << "The selected block will be red; all other blocks will be yellow.\n"
            << "Press [1] -> [3] to attach a new block to the selected block.\n"
            << "[1]: Attach using a WeldJoint\n"
            << "[2]: Attach using a RevoluteJoint\n"
            << "[3]: Attach using a BallJoint\n"
            << "The longitudinal direction of the new block will be along the x-axis (Red) of the target.\n"
            << "The joint axis will follow the z-axis (Blue) of the target when making a RevolueJoint.\n"
            << "Press [Delete] to permanently remove a block and all of its children.\n"
            << "Adding a block when nothing is currently selected will attach it to the world, beginning a new tree.\n"
            << "\n --- While Simulation is Active ---\n"
            << "The selected block will be Fuchsia; all other blocks will be blue.\n"
            << "Move around the target to pull on the selected block.\n"
            << "Press [Up] or [Down] to adjust the pulling strength.\n"
            << "Press [G] to toggle Gravity\n"
            << "Blocks belonging to different trees can collide with each other during simulation.\n"
            << "Collisions between blocks belonging to the same tree will be ignored.\n"
            << std::endl;

  // Run the GUI application
  viewer.run();
}
