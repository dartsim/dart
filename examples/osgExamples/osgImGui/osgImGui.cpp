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
#include <dart/external/imgui/imgui.h>

//==============================================================================
class CustomWorldNode : public dart::gui::osg::WorldNode
{
public:

  CustomWorldNode(const dart::simulation::WorldPtr& world = nullptr)
    : dart::gui::osg::WorldNode(world)
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
class TestWidget : public dart::gui::osg::ImGuiWidget
{
public:
  /// Constructor
  TestWidget(
      dart::gui::osg::ImGuiViewer* viewer, dart::simulation::WorldPtr world)
    : mViewer(viewer),
      mWorld(std::move(world)),
      mGuiGravity(true),
      mGravity(true),
      mGuiHeadlights(true)
  {
    // Do nothing
  }

  // Documentation inherited
  void render() override
  {
    ImGui::SetNextWindowPos(ImVec2(10,20));
    if (!ImGui::Begin("Tinkertoy Control", nullptr, ImVec2(240, 320), 0.5f,
                      ImGuiWindowFlags_NoResize |
                      ImGuiWindowFlags_MenuBar |
                      ImGuiWindowFlags_HorizontalScrollbar))
    {
      // Early out if the window is collapsed, as an optimization.
      ImGui::End();
      return;
    }

    // Menu
    if (ImGui::BeginMenuBar())
    {
      if (ImGui::BeginMenu("Menu"))
      {
        if (ImGui::MenuItem("Exit"))
          mViewer->setDone(true);
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Help"))
      {
        if (ImGui::MenuItem("About DART"))
          mViewer->showAbout();
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }

    ImGui::Text("An empty OSG example with ImGui");
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen))
    {
      int e = mViewer->isSimulating() ? 0 : 1;
      if(mViewer->isAllowingSimulation())
      {
        if (ImGui::RadioButton("Play", &e, 0) && !mViewer->isSimulating())
          mViewer->simulate(true);
        ImGui::SameLine();
        if (ImGui::RadioButton("Pause", &e, 1) && mViewer->isSimulating())
          mViewer->simulate(false);
      }

      ImGui::Text("Time: %.3f", mWorld->getTime());
    }

    if (ImGui::CollapsingHeader("World Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
      // Gravity
      ImGui::Checkbox("Gravity On/Off", &mGuiGravity);
      setGravity(mGuiGravity);

      ImGui::Spacing();

      // Headlights
      mGuiHeadlights = mViewer->checkHeadlights();
      ImGui::Checkbox("Headlights On/Off", &mGuiHeadlights);
      mViewer->switchHeadlights(mGuiHeadlights);
    }

    if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen))
    {
      osg::Vec3d eye;
      osg::Vec3d center;
      osg::Vec3d up;
      mViewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);

      ImGui::Text("Eye   : (%.2f, %.2f, %.2f)", eye.x(), eye.y(), eye.z());
      ImGui::Text("Center: (%.2f, %.2f, %.2f)", center.x(), center.y(), center.z());
      ImGui::Text("Up    : (%.2f, %.2f, %.2f)", up.x(), up.y(), up.z());
    }

    if (ImGui::CollapsingHeader("Help"))
    {
      ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 320);
      ImGui::Text("User Guide:\n");
      ImGui::Text("%s", mViewer->getInstructions().c_str());
      ImGui::PopTextWrapPos();
    }

    ImGui::End();
  }

protected:
  void setGravity(bool gravity)
  {
    if (mGravity == gravity)
      return;

    mGravity = gravity;

    if (mGravity)
      mWorld->setGravity(-9.81*Eigen::Vector3d::UnitZ());
    else
      mWorld->setGravity(Eigen::Vector3d::Zero());
  }

  dart::gui::osg::ImGuiViewer* mViewer;
  dart::simulation::WorldPtr mWorld;
  bool mGuiGravity;
  bool mGravity;
  bool mGuiHeadlights;
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
  dart::gui::osg::ImGuiViewer viewer;
  viewer.addWorldNode(node);

  // Add control widget for atlas
  viewer.getImGuiHandler()->addWidget(
      std::make_shared<TestWidget>(&viewer, world));

  // Active the drag-and-drop feature for the target
  viewer.enableDragAndDrop(target.get());

  // Pass in the custom event handler
  viewer.addEventHandler(new CustomEventHandler);

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3( 2.57f,  3.14f, 1.64f),
        ::osg::Vec3( 0.00f,  0.00f, 0.00f),
        ::osg::Vec3(-0.24f, -0.25f, 0.94f));
  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();
}
