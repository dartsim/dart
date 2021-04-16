/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <dart/collision/bullet/bullet.hpp>
#include <dart/dart.hpp>
#include <dart/external/imgui/imgui.h>
#include <dart/gui/osg/osg.hpp>
#include <dart/io/io.hpp>

using namespace dart;

class FetchWorldNode : public gui::osg::RealTimeWorldNode
{
public:
  explicit FetchWorldNode(
      simulation::WorldPtr world,
      dynamics::SkeletonPtr robot,
      dynamics::BodyNode* mocap,
      dynamics::Frame* interactiveFrame)
    : gui::osg::RealTimeWorldNode(std::move(world)),
      mRobot(std::move(robot)),
      mMocap(mocap),
      mInteractiveFrame(interactiveFrame)
  {
    // Do nothing
  }

  void customPreStep() override
  {
    if (!mRobot || !mMocap || !mInteractiveFrame)
      return;

    mMocap->getParentJoint()->setTransformFromParentBodyNode(
        mInteractiveFrame->getTransform());
  }

protected:
  dynamics::SkeletonPtr mRobot;
  dynamics::BodyNode* mMocap;
  dynamics::Frame* mInteractiveFrame;
};

class PointCloudWidget : public dart::gui::osg::ImGuiWidget
{
public:
  PointCloudWidget(
      dart::gui::osg::ImGuiViewer* viewer,
      FetchWorldNode* node,
      gui::osg::GridVisual* grid)
    : mViewer(viewer), mNode(node), mGrid(grid)
  {
    // Do nothing
  }

  void render() override
  {
    ImGui::SetNextWindowPos(ImVec2(10, 20));
    ImGui::SetNextWindowSize(ImVec2(360, 600));
    ImGui::SetNextWindowBgAlpha(0.5f);
    if (!ImGui::Begin(
            "Fetch robot example",
            nullptr,
            ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_HorizontalScrollbar))
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

    ImGui::Text("Point cloud and voxel grid rendering example");
    ImGui::Spacing();
    ImGui::TextWrapped(
        "The whole body motion of the Fetch robot is determined by the "
        "location of the end-effector. The end-effector simply follows the "
        "invisible dummy object where the position is indicated at the cross "
        "of the two tranparent green bars.");

    if (ImGui::CollapsingHeader("Help"))
    {
      ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 320);
      ImGui::Text("User Guid:\n");
      ImGui::Text("%s", mViewer->getInstructions().c_str());
      ImGui::PopTextWrapPos();
    }

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
    }

    ImGui::End();
  }

protected:
  osg::ref_ptr<dart::gui::osg::ImGuiViewer> mViewer;
  osg::ref_ptr<FetchWorldNode> mNode;
  osg::ref_ptr<gui::osg::GridVisual> mGrid;
};

int main()
{
  using namespace math::suffixes;

  // Create a world from ant.xml
  auto world = io::MjcfParser::readWorld(
      "dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml");
  assert(world);
  world->getConstraintSolver()->setCollisionDetector(
      collision::BulletCollisionDetector::create());

  // Get Fetch robot and set properties
  auto robot = world->getSkeleton("robot0:base_link");
  assert(robot);
  robot->getJoint(0)->setActuatorType(dynamics::Joint::ActuatorType::LOCKED);
  for (auto i = 0u; i < robot->getNumDofs(); ++i)
  {
    robot->getDof(i)->setSpringStiffness(1e+3);
  }
  robot->getJoint("robot0:torso_lift_joint")->setSpringStiffness(0, 1e+7);

  // Set initial base positions
  robot->setPosition(0, 0.405);
  robot->setPosition(1, 0.480);
  robot->setPosition(2, 0.000);

  // Set initial arm positions;
  robot->setPosition(6, 0.01);
  robot->setPosition(7, -0.73);
  robot->setPosition(8, 0.00);
  robot->setPosition(9, 1.64);
  robot->setPosition(10, 0.0);
  robot->setPosition(11, 0.66);
  robot->setPosition(12, 0.01);

  // Get object and set initial object transform
  auto object = world->getSkeleton("object0");
  assert(object);
  object->setPosition(3, 1.25);
  object->setPosition(4, 0.53);
  object->setPosition(5, 0.40);

  // Get mocap object
  auto mocap = world->getSkeleton("robot0:mocap");
  assert(mocap);

  // Reset the relative transform constraint of mocap object and fetch's EE
  auto weldJointConstraint
      = std::dynamic_pointer_cast<dynamics::WeldJointConstraint>(
          world->getConstraintSolver()->getConstraint(0));
  assert(weldJointConstraint);
  weldJointConstraint->setRelativeTransform(Eigen::Isometry3d::Identity());

  // Create interactive frame to control the EE of Fetch
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(1.3, 0.75, 0.50);
  tf.linear()
      = Eigen::AngleAxisd(0.5_pi, Eigen::Vector3d::UnitY()).toRotationMatrix();
  auto frame = std::make_shared<gui::osg::InteractiveFrame>(
      dynamics::Frame::World(), "interactive frame", tf, 0.2);
  world->addSimpleFrame(frame);

  // Wrap a WorldNode around it
  ::osg::ref_ptr<FetchWorldNode> node
      = new FetchWorldNode(world, robot, mocap->getRootBodyNode(), frame.get());

  // Create a Viewer and set it up with the WorldNode
  auto viewer = gui::osg::ImGuiViewer();
  viewer.addWorldNode(node);

  // Create grid
  ::osg::ref_ptr<gui::osg::GridVisual> grid = new gui::osg::GridVisual();
  grid->setOffset(Eigen::Vector3d(1.3, 0.75, 0));
  viewer.addAttachment(grid);

  // Add control widget for fetch
  viewer.getImGuiHandler()->addWidget(
      std::make_shared<PointCloudWidget>(&viewer, node, grid));

  viewer.enableDragAndDrop(frame.get());

  // Set up the window to be 1280x960
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(4.0f, 4.0f, 2.5f),
      ::osg::Vec3(0.1f, -0.3f, 0.3f),
      ::osg::Vec3(0.f, 0.f, 1.f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
