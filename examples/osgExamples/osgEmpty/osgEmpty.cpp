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

//==============================================================================
class CustomWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:

  CustomWorldNode(const dart::simulation::WorldPtr& world = nullptr)
    : dart::gui::osg::RealTimeWorldNode(world)
  {
    // Set up the customized WorldNode
  }

  void customPreRefresh()
  {
    // Use this function to execute custom code before each time that the
    // window is rendered. This function can be deleted if it does not need
    // to be used.
  }

  void customPostRefresh()
  {
    // Use this function to execute custom code after each time that the
    // window is rendered. This function can be deleted if it does not need
    // to be used.
  }

  void customPreStep()
  {
    // Use this function to execute custom code before each simulation time
    // step is performed. This function can be deleted if it does not need
    // to be used.
  }

  void customPostStep()
  {
    // Use this function to execute custom code after each simulation time
    // step is performed. This function can be deleted if it does not need
    // to be used.
  }

};

//==============================================================================
class CustomEventHandler : public osgGA::GUIEventHandler
{
public:

  CustomEventHandler(/*Pass in any necessary arguments*/)
  {
    // Set up the customized event handler
  }

  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter&) override
  {
    if(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
    {
      if(ea.getKey() == 'q')
      {
        std::cout << "Lowercase q pressed" << std::endl;
        return true;
      }
      else if(ea.getKey() == 'Q')
      {
        std::cout << "Capital Q pressed" << std::endl;
        return true;
      }
      else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
      {
        std::cout << "Left arrow key pressed" << std::endl;
        return true;
      }
      else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
      {
        std::cout << "Right arrow key pressed" << std::endl;
        return true;
      }
    }
    else if(ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
    {
      if(ea.getKey() == 'q')
      {
        std::cout << "Lowercase q released" << std::endl;
        return true;
      }
      else if(ea.getKey() == 'Q')
      {
        std::cout << "Capital Q released" << std::endl;
        return true;
      }
      else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
      {
        std::cout << "Left arrow key released" << std::endl;
        return true;
      }
      else if(ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
      {
        std::cout << "Right arrow key released" << std::endl;
        return true;
      }
    }

    // The return value should be 'true' if the input has been fully handled
    // and should not be visible to any remaining event handlers. It should be
    // false if the input has not been fully handled and should be viewed by
    // any remaining event handlers.
    return false;
  }

};

//==============================================================================
int main()
{
  // Create a world
  dart::simulation::WorldPtr world(new dart::simulation::World);

  // Add a target object to the world
  dart::gui::osg::InteractiveFramePtr target(
        new dart::gui::osg::InteractiveFrame(dart::dynamics::Frame::World()));
  world->addSimpleFrame(target);

  // Wrap a WorldNode around it
  osg::ref_ptr<CustomWorldNode> node = new CustomWorldNode(world);

  // Create a Viewer and set it up with the WorldNode
  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(node);

  // Active the drag-and-drop feature for the target
  viewer.enableDragAndDrop(target.get());

  // Pass in the custom event handler
  viewer.addEventHandler(new CustomEventHandler);

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(::osg::Vec3( 2.57,  3.14, 1.64),
                                                 ::osg::Vec3( 0.00,  0.00, 0.00),
                                                 ::osg::Vec3(-0.24, -0.25, 0.94));
  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();
}


