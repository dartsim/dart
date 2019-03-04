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

#include <iostream>

#include <dart/dart.hpp>
#include <dart/external/imgui/imgui.h>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>

using namespace dart;

//==============================================================================
dynamics::SkeletonPtr createBox(const Eigen::Vector3d& position)
{
  dynamics::SkeletonPtr boxSkel = dynamics::Skeleton::create("box");

  // Give the floor a body
  dynamics::BodyNodePtr boxBody
      = boxSkel->createJointAndBodyNodePair<dynamics::FreeJoint>(nullptr)
            .second;

  // Give the body a shape
  double boxWidth = 1.0;
  double boxDepth = 1.0;
  double boxHeight = 0.5;
  auto boxShape = std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d(boxWidth, boxDepth, boxHeight));
  dynamics::ShapeNode* shapeNode = boxBody->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(
      dart::math::Random::uniform<Eigen::Vector3d>(0.0, 1.0));

  // Put the body into position
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  boxBody->getParentJoint()->setTransformFromParentBodyNode(tf);

  return boxSkel;
}

//==============================================================================
std::vector<dynamics::SkeletonPtr> createBoxStack(
    std::size_t numBoxes, double heightFromGround = 0.5)
{
  std::vector<dynamics::SkeletonPtr> boxSkels(numBoxes);

  for (auto i = 0u; i < numBoxes; ++i)
    boxSkels[i] = createBox(
        Eigen::Vector3d(0.0, 0.0, heightFromGround + 0.25 + i * 0.5));

  return boxSkels;
}

//==============================================================================
dynamics::SkeletonPtr createFloor()
{
  dynamics::SkeletonPtr floor = dynamics::Skeleton::create("floor");

  // Give the floor a body
  dynamics::BodyNodePtr body
      = floor->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr).second;

  // Give the body a shape
  double floorWidth = 10.0;
  double floorHeight = 0.01;
  auto box = std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d(floorWidth, floorWidth, floorHeight));
  dynamics::ShapeNode* shapeNode = body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::LightGray());

  // Put the body into position
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floorHeight / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

//==============================================================================
class CustomWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  explicit CustomWorldNode(const dart::simulation::WorldPtr& world = nullptr)
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

  virtual bool handle(
      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
    {
      if (ea.getKey() == 'q')
      {
        std::cout << "Lowercase q pressed" << std::endl;
        return true;
      }
      else if (ea.getKey() == 'Q')
      {
        std::cout << "Capital Q pressed" << std::endl;
        return true;
      }
      else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
      {
        std::cout << "Left arrow key pressed" << std::endl;
        return true;
      }
      else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
      {
        std::cout << "Right arrow key pressed" << std::endl;
        return true;
      }
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP)
    {
      if (ea.getKey() == 'q')
      {
        std::cout << "Lowercase q released" << std::endl;
        return true;
      }
      else if (ea.getKey() == 'Q')
      {
        std::cout << "Capital Q released" << std::endl;
        return true;
      }
      else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
      {
        std::cout << "Left arrow key released" << std::endl;
        return true;
      }
      else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
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
      mGuiHeadlights(true),
      mSolverType(-1)
  {
    // Do nothing
  }

  // Documentation inherited
  void render() override
  {
    ImGui::SetNextWindowPos(ImVec2(10, 20));
    if (!ImGui::Begin(
            "Box Stacking",
            nullptr,
            ImVec2(240, 320),
            0.5f,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar
                | ImGuiWindowFlags_HorizontalScrollbar))
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

    ImGui::Text("Box stacking demo");
    ImGui::Spacing();

    ImGui::Separator();
    ImGui::Text(
        "%.3f ms/frame (%.1f FPS)",
        1000.0f / ImGui::GetIO().Framerate,
        ImGui::GetIO().Framerate);
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen))
    {
      int e = mViewer->isSimulating() ? 0 : 1;
      if (mViewer->isAllowingSimulation())
      {
        if (ImGui::RadioButton("Play", &e, 0) && !mViewer->isSimulating())
          mViewer->simulate(true);
        ImGui::SameLine();
        if (ImGui::RadioButton("Pause", &e, 1) && mViewer->isSimulating())
          mViewer->simulate(false);
      }

      ImGui::Text("LCP solver:");

      static int solverType = 0;
      // ImGui::RadioButton("SI", &solverType, 0);
      ImGui::RadioButton("Dantzig", &solverType, 0);
      ImGui::SameLine();
      ImGui::RadioButton("Dantzig", &solverType, 1);
      ImGui::SameLine();
      ImGui::RadioButton("PGS", &solverType, 2);
      setLcpSolver(solverType);

      ImGui::Text("Time: %.3f", mWorld->getTime());
    }

    if (ImGui::CollapsingHeader(
            "World Options", ImGuiTreeNodeFlags_DefaultOpen))
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
      ImGui::Text(
          "Center: (%.2f, %.2f, %.2f)", center.x(), center.y(), center.z());
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
  void setLcpSolver(int solverType)
  {
    if (solverType == mSolverType)
      return;

    if (solverType == 0)
    {
      // auto solver
      //     =
      //     common::make_unique<constraint::SequentialImpulseConstraintSolver>(
      //         mWorld->getTimeStep());
      auto lcpSolver = std::make_shared<constraint::DantzigBoxedLcpSolver>();
      auto solver = common::make_unique<constraint::BoxedLcpConstraintSolver>(
          mWorld->getTimeStep(), lcpSolver);
      mWorld->setConstraintSolver(std::move(solver));
    }
    else if (solverType == 1)
    {
      auto lcpSolver = std::make_shared<constraint::DantzigBoxedLcpSolver>();
      auto solver = common::make_unique<constraint::BoxedLcpConstraintSolver>(
          mWorld->getTimeStep(), lcpSolver);
      mWorld->setConstraintSolver(std::move(solver));
    }
    else if (solverType == 2)
    {
      auto lcpSolver = std::make_shared<constraint::PgsBoxedLcpSolver>();
      auto solver = common::make_unique<constraint::BoxedLcpConstraintSolver>(
          mWorld->getTimeStep(), lcpSolver);
      mWorld->setConstraintSolver(std::move(solver));
    }
    else
    {
      dtwarn << "Unsupported boxed-LCP solver selected: " << solverType << "\n";
    }

    mSolverType = solverType;
  }

  void setGravity(bool gravity)
  {
    if (mGravity == gravity)
      return;

    mGravity = gravity;

    if (mGravity)
      mWorld->setGravity(-9.81 * Eigen::Vector3d::UnitZ());
    else
      mWorld->setGravity(Eigen::Vector3d::Zero());
  }

  dart::gui::osg::ImGuiViewer* mViewer;
  dart::simulation::WorldPtr mWorld;
  bool mGuiGravity;
  bool mGravity;
  bool mGuiHeadlights;
  int mSolverType;
};

//==============================================================================
int main()
{
  simulation::WorldPtr world = simulation::World::create();
  world->addSkeleton(createFloor());

  auto boxSkels = createBoxStack(5);
  for (const auto& boxSkel : boxSkels)
    world->addSkeleton(boxSkel);

  // Wrap a WorldNode around it
  osg::ref_ptr<CustomWorldNode> node = new CustomWorldNode(world);

  // Create a Viewer and set it up with the WorldNode
  dart::gui::osg::ImGuiViewer viewer;
  viewer.addWorldNode(node);

  // Add control widget for atlas
  viewer.getImGuiHandler()->addWidget(
      std::make_shared<TestWidget>(&viewer, world));

  // Pass in the custom event handler
  viewer.addEventHandler(new CustomEventHandler);

  // Set up the window to be 800x640
  viewer.setUpViewInWindow(0, 0, 800, 640);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(12.00f, 12.00f, 9.00f),
      ::osg::Vec3(0.00f, 0.00f, 2.00f),
      ::osg::Vec3(0.00f, 0.00f, 1.00f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
