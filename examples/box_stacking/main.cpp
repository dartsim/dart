/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/gui/All.hpp>
#include <dart/gui/IncludeImGui.hpp>

#include <dart/utils/All.hpp>

#include <dart/math/lcp/pivoting/DantzigSolver.hpp>
#include <dart/math/lcp/projection/PgsSolver.hpp>

#include <dart/All.hpp>

#include <CLI/CLI.hpp>

#include <iostream>

using namespace dart;

//==============================================================================
dynamics::SkeletonPtr createBox(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& offset = Eigen::Vector3d::Zero())
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
  tf.translation() = position + offset;
  boxBody->getParentJoint()->setTransformFromParentBodyNode(tf);

  return boxSkel;
}

//==============================================================================
std::vector<dynamics::SkeletonPtr> createBoxStack(
    std::size_t numBoxes,
    const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
    double heightFromGround = 0.5)
{
  std::vector<dynamics::SkeletonPtr> boxSkels(numBoxes);

  for (auto i = 0u; i < numBoxes; ++i)
    boxSkels[i] = createBox(
        Eigen::Vector3d(0.0, 0.0, heightFromGround + 0.25 + i * 0.5), offset);

  return boxSkels;
}

//==============================================================================
dynamics::SkeletonPtr createFloor(
    const Eigen::Vector3d& offset = Eigen::Vector3d::Zero())
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
  tf.translation() = offset + Eigen::Vector3d(0.0, 0.0, -floorHeight / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

//==============================================================================
void populateWorld(
    const simulation::WorldPtr& world,
    const Eigen::Vector3d& offset,
    std::size_t numBoxes)
{
  world->removeAllSkeletons();
  world->addSkeleton(createFloor(offset));

  auto boxSkels = createBoxStack(numBoxes, offset);
  for (const auto& boxSkel : boxSkels)
    world->addSkeleton(boxSkel);

  world->reset();
}

//==============================================================================
class CustomWorldNode : public dart::gui::RealTimeWorldNode
{
public:
  explicit CustomWorldNode(const dart::simulation::WorldPtr& world = nullptr)
    : dart::gui::RealTimeWorldNode(world)
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
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
      if (ea.getKey() == 'q') {
        std::cout << "Lowercase q pressed" << std::endl;
        return true;
      } else if (ea.getKey() == 'Q') {
        std::cout << "Capital Q pressed" << std::endl;
        return true;
      } else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Left) {
        std::cout << "Left arrow key pressed" << std::endl;
        return true;
      } else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Right) {
        std::cout << "Right arrow key pressed" << std::endl;
        return true;
      }
    } else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {
      if (ea.getKey() == 'q') {
        std::cout << "Lowercase q released" << std::endl;
        return true;
      } else if (ea.getKey() == 'Q') {
        std::cout << "Capital Q released" << std::endl;
        return true;
      } else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Left) {
        std::cout << "Left arrow key released" << std::endl;
        return true;
      } else if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Right) {
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
class TestWidget : public dart::gui::ImGuiWidget
{
public:
  /// Constructor
  TestWidget(
      dart::gui::ImGuiViewer* viewer,
      dart::simulation::WorldPtr worldLeft,
      dart::simulation::WorldPtr worldRight,
      const Eigen::Vector3d& leftOffset,
      const Eigen::Vector3d& rightOffset,
      std::size_t numBoxes)
    : mViewer(viewer),
      mWorldLeft(std::move(worldLeft)),
      mWorldRight(std::move(worldRight)),
      mLeftOffset(leftOffset),
      mRightOffset(rightOffset),
      mNumBoxes(numBoxes),
      mGuiGravity(true),
      mGravity(true),
      mGuiHeadlights(true),
      mSplitImpulseEnabled(false),
      mSolverType(-1),
      mManifoldEnabledLeft(false),
      mManifoldEnabledRight(true)
  {
    applyContactManifoldSetting(mWorldLeft, mManifoldEnabledLeft);
    applyContactManifoldSetting(mWorldRight, mManifoldEnabledRight);

    if (auto* solver = mWorldLeft ? mWorldLeft->getConstraintSolver() : nullptr)
      mSplitImpulseEnabled = solver->isSplitImpulseEnabled();
    else if (
        auto* solver
        = mWorldRight ? mWorldRight->getConstraintSolver() : nullptr)
      mSplitImpulseEnabled = solver->isSplitImpulseEnabled();

    setSplitImpulse(mSplitImpulseEnabled);
  }

  // Documentation inherited
  void render() override
  {
    ImGui::SetNextWindowPos(ImVec2(10, 20));
    ImGui::SetNextWindowSize(ImVec2(240, 320));
    ImGui::SetNextWindowBgAlpha(0.5f);
    if (!ImGui::Begin(
            "Box Stacking",
            nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar
                | ImGuiWindowFlags_HorizontalScrollbar)) {
      // Early out if the window is collapsed, as an optimization.
      ImGui::End();
      return;
    }

    // Menu
    if (ImGui::BeginMenuBar()) {
      if (ImGui::BeginMenu("Menu")) {
        if (ImGui::MenuItem("Exit"))
          mViewer->setDone(true);
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Help")) {
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

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      int e = mViewer->isSimulating() ? 0 : 1;
      if (mViewer->isAllowingSimulation()) {
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

      if (ImGui::Checkbox("Split impulse", &mSplitImpulseEnabled)) {
        setSplitImpulse(mSplitImpulseEnabled);
      }

      const auto time = mWorldLeft
                            ? mWorldLeft->getTime()
                            : (mWorldRight ? mWorldRight->getTime() : 0.0);
      ImGui::Text("Time: %.3f", time);
    }

    if (ImGui::CollapsingHeader(
            "World Options", ImGuiTreeNodeFlags_DefaultOpen)) {
      // Gravity
      ImGui::Checkbox("Gravity On/Off", &mGuiGravity);
      setGravity(mGuiGravity);

      ImGui::Spacing();

      // Headlights
      mGuiHeadlights = mViewer->checkHeadlights();
      ImGui::Checkbox("Headlights On/Off", &mGuiHeadlights);
      mViewer->switchHeadlights(mGuiHeadlights);
    }

    if (ImGui::CollapsingHeader("Contact Manifold Cache")) {
      bool manifoldLeft = mManifoldEnabledLeft;
      bool manifoldRight = mManifoldEnabledRight;

      ImGui::Checkbox("Left world (legacy)", &manifoldLeft);
      ImGui::Checkbox("Right world (persistent)", &manifoldRight);

      if (manifoldLeft != mManifoldEnabledLeft) {
        mManifoldEnabledLeft = manifoldLeft;
        applyContactManifoldSetting(mWorldLeft, mManifoldEnabledLeft);
      }

      if (manifoldRight != mManifoldEnabledRight) {
        mManifoldEnabledRight = manifoldRight;
        applyContactManifoldSetting(mWorldRight, mManifoldEnabledRight);
      }

      if (ImGui::Button("Reset stacks")) {
        populateWorld(mWorldLeft, mLeftOffset, mNumBoxes);
        populateWorld(mWorldRight, mRightOffset, mNumBoxes);
        applyContactManifoldSetting(mWorldLeft, mManifoldEnabledLeft);
        applyContactManifoldSetting(mWorldRight, mManifoldEnabledRight);
      }

      ImGui::Spacing();
      ImGui::Text(
          "Left: manifolds %zu, persistent contacts %zu, constraints %zu",
          getManifoldCount(mWorldLeft),
          getPersistentCount(mWorldLeft),
          getContactConstraintCount(mWorldLeft));
      ImGui::Text(
          "Right: manifolds %zu, persistent contacts %zu, constraints %zu",
          getManifoldCount(mWorldRight),
          getPersistentCount(mWorldRight),
          getContactConstraintCount(mWorldRight));
    }

    if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
      osg::Vec3d eye;
      osg::Vec3d center;
      osg::Vec3d up;
      mViewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);

      ImGui::Text("Eye   : (%.2f, %.2f, %.2f)", eye.x(), eye.y(), eye.z());
      ImGui::Text(
          "Center: (%.2f, %.2f, %.2f)", center.x(), center.y(), center.z());
      ImGui::Text("Up    : (%.2f, %.2f, %.2f)", up.x(), up.y(), up.z());
    }

    if (ImGui::CollapsingHeader("Help")) {
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

    for (const auto& world : {mWorldLeft, mWorldRight}) {
      if (!world)
        continue;

      if (solverType == 0) {
        auto lcpSolver = std::make_shared<math::DantzigSolver>();
        auto solver = std::make_unique<constraint::ConstraintSolver>(lcpSolver);
        solver->setSplitImpulseEnabled(mSplitImpulseEnabled);
        world->setConstraintSolver(std::move(solver));
      } else if (solverType == 1) {
        auto lcpSolver = std::make_shared<math::DantzigSolver>();
        auto solver = std::make_unique<constraint::ConstraintSolver>(lcpSolver);
        solver->setSplitImpulseEnabled(mSplitImpulseEnabled);
        world->setConstraintSolver(std::move(solver));
      } else if (solverType == 2) {
        auto lcpSolver = std::make_shared<math::PgsSolver>();
        auto solver = std::make_unique<constraint::ConstraintSolver>(lcpSolver);
        solver->setSplitImpulseEnabled(mSplitImpulseEnabled);
        world->setConstraintSolver(std::move(solver));
      } else {
        DART_WARN("Unsupported boxed-LCP solver selected: {}", solverType);
      }

      applyContactManifoldSetting(world, getManifoldEnabled(world));
    }

    mSolverType = solverType;
  }

  void setGravity(bool gravity)
  {
    if (mGravity == gravity)
      return;

    mGravity = gravity;

    for (const auto& world : {mWorldLeft, mWorldRight}) {
      if (!world)
        continue;

      if (mGravity)
        world->setGravity(-9.81 * Eigen::Vector3d::UnitZ());
      else
        world->setGravity(Eigen::Vector3d::Zero());
    }
  }

  bool getManifoldEnabled(const simulation::WorldPtr& world) const
  {
    if (world == mWorldLeft)
      return mManifoldEnabledLeft;
    if (world == mWorldRight)
      return mManifoldEnabledRight;
    return false;
  }

  void applyContactManifoldSetting(
      const simulation::WorldPtr& world, bool enabled) const
  {
    if (!world)
      return;

    auto* solver = world->getConstraintSolver();
    if (!solver)
      return;

    solver->setContactManifoldCacheEnabled(enabled);
  }

  std::size_t getManifoldCount(const simulation::WorldPtr& world) const
  {
    auto* solver = world ? world->getConstraintSolver() : nullptr;
    return solver ? solver->getNumContactManifolds() : 0u;
  }

  std::size_t getPersistentCount(const simulation::WorldPtr& world) const
  {
    auto* solver = world ? world->getConstraintSolver() : nullptr;
    return solver ? solver->getNumPersistentContacts() : 0u;
  }

  std::size_t getContactConstraintCount(const simulation::WorldPtr& world) const
  {
    auto* solver = world ? world->getConstraintSolver() : nullptr;
    return solver ? solver->getNumContactConstraints() : 0u;
  }

  void setSplitImpulse(bool enabled)
  {
    for (const auto& world : {mWorldLeft, mWorldRight}) {
      if (!world)
        continue;

      auto* solver = world->getConstraintSolver();
      if (!solver)
        continue;

      solver->setSplitImpulseEnabled(enabled);
    }
  }

  ::osg::ref_ptr<dart::gui::ImGuiViewer> mViewer;
  dart::simulation::WorldPtr mWorldLeft;
  dart::simulation::WorldPtr mWorldRight;
  Eigen::Vector3d mLeftOffset;
  Eigen::Vector3d mRightOffset;
  std::size_t mNumBoxes;
  bool mGuiGravity;
  bool mGravity;
  bool mGuiHeadlights;
  bool mSplitImpulseEnabled;
  int mSolverType;
  bool mManifoldEnabledLeft;
  bool mManifoldEnabledRight;
};

//==============================================================================
int main(int argc, char* argv[])
{
  CLI::App app("Box stacking example");
  double guiScale = 1.0;
  app.add_option("--gui-scale", guiScale, "Scale factor for ImGui widgets")
      ->check(CLI::PositiveNumber);
  CLI11_PARSE(app, argc, argv);

  const Eigen::Vector3d leftOffset(-3.0, 0.0, 0.0);
  const Eigen::Vector3d rightOffset(3.0, 0.0, 0.0);
  constexpr std::size_t numBoxes = 5u;

  simulation::WorldPtr worldLeft = simulation::World::create();
  simulation::WorldPtr worldRight = simulation::World::create();

  populateWorld(worldLeft, leftOffset, numBoxes);
  populateWorld(worldRight, rightOffset, numBoxes);

  if (auto* solver = worldRight->getConstraintSolver())
    solver->setContactManifoldCacheEnabled(true);

  // Wrap a WorldNode around it
  osg::ref_ptr<CustomWorldNode> nodeLeft = new CustomWorldNode(worldLeft);
  osg::ref_ptr<CustomWorldNode> nodeRight = new CustomWorldNode(worldRight);

  // Create a Viewer and set it up with the WorldNode
  osg::ref_ptr<dart::gui::ImGuiViewer> viewer = new dart::gui::ImGuiViewer();
  viewer->setImGuiScale(static_cast<float>(guiScale));
  viewer->addWorldNode(nodeLeft);
  viewer->addWorldNode(nodeRight);

  // Add control widget for atlas
  viewer->getImGuiHandler()->addWidget(
      std::make_shared<TestWidget>(
          viewer, worldLeft, worldRight, leftOffset, rightOffset, numBoxes));

  // Pass in the custom event handler
  viewer->addEventHandler(new CustomEventHandler);

  // Set up the window to be 800x640
  viewer->setUpViewInWindow(0, 0, 800, 640);

  // Adjust the viewpoint of the Viewer
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(12.00f, 12.00f, 9.00f),
      ::osg::Vec3(0.00f, 0.00f, 2.00f),
      ::osg::Vec3(0.00f, 0.00f, 1.00f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  // Begin running the application loop
  viewer->run();

  return 0;
}
