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

#include <dart/gui/osg/osg.hpp>

#include <dart/constraint/BallJointConstraint.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/constraint/CylindricalJointConstraint.hpp>
#include <dart/constraint/WeldJointConstraint.hpp>

#include <dart/dart.hpp>

#include <osg/Geode>
#include <osg/Group>
#include <osg/StateSet>
#include <osgText/Text>

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>

namespace {

constexpr double kBodyMass = 1.0;
constexpr int kViewerWidth = 960;
constexpr int kViewerHeight = 640;
constexpr float kPanelWidth = 430.0f;
constexpr float kPanelHeight = 290.0f;

struct DemoBody
{
  dart::dynamics::SkeletonPtr skeleton;
  dart::dynamics::FreeJoint* joint;
  dart::dynamics::BodyNode* body;
};

Eigen::Isometry3d makeTransform(const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  return transform;
}

void setFreeJointVelocity(
    dart::dynamics::FreeJoint* joint,
    const Eigen::Vector3d& angular,
    const Eigen::Vector3d& linear)
{
  Eigen::Matrix<double, 6, 1> velocity = Eigen::Matrix<double, 6, 1>::Zero();
  velocity.head<3>() = angular;
  velocity.tail<3>() = linear;
  joint->setVelocities(velocity);
}

DemoBody createFreeBox(
    const std::string& name,
    const Eigen::Isometry3d& pose,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& color,
    const Eigen::Vector3d& visualOffset = Eigen::Vector3d::Zero())
{
  DemoBody result;
  result.skeleton = dart::dynamics::Skeleton::create(name);
  auto pair = result.skeleton
                  ->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  result.joint = pair.first;
  result.body = pair.second;

  result.joint->setPositions(
      dart::dynamics::FreeJoint::convertToPositions(pose));
  result.body->setMass(kBodyMass);

  for (std::size_t i = 0; i < result.joint->getNumDofs(); ++i)
    result.joint->setDampingCoefficient(i, 0.02);

  auto shape = std::make_shared<dart::dynamics::BoxShape>(size);
  auto shapeNode = result.body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->setRelativeTranslation(visualOffset);
  shapeNode->getVisualAspect()->setColor(color);

  return result;
}

dart::dynamics::SimpleFramePtr createSphereMarker(
    const std::string& name,
    const Eigen::Vector3d& position,
    double radius,
    const Eigen::Vector3d& color)
{
  auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(), name, makeTransform(position));
  frame->setShape(std::make_shared<dart::dynamics::SphereShape>(radius));
  frame->createVisualAspect();
  frame->getVisualAspect()->setColor(color);
  return frame;
}

dart::dynamics::SimpleFramePtr createBoxMarker(
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& color)
{
  auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(), name, makeTransform(position));
  frame->setShape(std::make_shared<dart::dynamics::BoxShape>(size));
  frame->createVisualAspect();
  frame->getVisualAspect()->setColor(color);
  return frame;
}

dart::dynamics::SimpleFramePtr createLineMarker(
    const std::string& name,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    float thickness,
    const Eigen::Vector3d& color)
{
  auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(), name);
  frame->setShape(std::make_shared<dart::dynamics::LineSegmentShape>(
      start, end, thickness));
  frame->createVisualAspect();
  frame->getVisualAspect()->setColor(color);
  return frame;
}

osg::ref_ptr<osg::Geode> createTextLabel(
    const std::string& text,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color,
    float characterSize)
{
  osg::ref_ptr<osgText::Text> label = new osgText::Text();
  label->setText(text);
  label->setPosition(osg::Vec3(position.x(), position.y(), position.z()));
  label->setColor(osg::Vec4(color.x(), color.y(), color.z(), 1.0));
  label->setCharacterSizeMode(osgText::TextBase::SCREEN_COORDS);
  label->setCharacterSize(characterSize);
  label->setAxisAlignment(osgText::TextBase::SCREEN);
  label->setAlignment(osgText::TextBase::CENTER_CENTER);
  label->setBackdropType(osgText::Text::OUTLINE);
  label->setBackdropColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
  label->setEnableDepthWrites(false);

  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  geode->addDrawable(label);
  geode->getOrCreateStateSet()->setMode(
      GL_DEPTH_TEST, osg::StateAttribute::OFF);
  return geode;
}

osg::ref_ptr<osg::Group> createSceneLabels(double guiScale)
{
  const float baseSize = static_cast<float>(18.0 * guiScale);
  const float titleSize = static_cast<float>(22.0 * guiScale);

  osg::ref_ptr<osg::Group> labels = new osg::Group();
  labels->addChild(createTextLabel(
      "DART 6 DynamicJointConstraint catalog",
      Eigen::Vector3d(0.0, -1.25, 3.35),
      Eigen::Vector3d(0.08, 0.08, 0.08),
      titleSize));
  labels->addChild(createTextLabel(
      "BALL\nfixed point\nfree rotation",
      Eigen::Vector3d(-3.0, -1.05, 2.55),
      Eigen::Vector3d(0.75, 0.12, 0.08),
      baseSize));
  labels->addChild(createTextLabel(
      "CYLINDRICAL\nrail fixed\nslide + spin",
      Eigen::Vector3d(0.0, -1.05, 2.75),
      Eigen::Vector3d(0.0, 0.45, 0.70),
      baseSize));
  labels->addChild(createTextLabel(
      "WELD\npose fixed\nno free DOF",
      Eigen::Vector3d(3.0, -1.05, 2.55),
      Eigen::Vector3d(0.12, 0.45, 0.12),
      baseSize));

  return labels;
}

void addSceneGuides(const dart::simulation::WorldPtr& world)
{
  world->addSimpleFrame(createBoxMarker(
      "ball_lane_pad",
      Eigen::Vector3d(-3.0, 0.0, 0.02),
      Eigen::Vector3d(1.9, 1.25, 0.04),
      Eigen::Vector3d(0.60, 0.20, 0.16)));
  world->addSimpleFrame(createBoxMarker(
      "cylindrical_lane_pad",
      Eigen::Vector3d(0.0, 0.0, 0.02),
      Eigen::Vector3d(1.9, 1.25, 0.04),
      Eigen::Vector3d(0.12, 0.42, 0.62)));
  world->addSimpleFrame(createBoxMarker(
      "weld_lane_pad",
      Eigen::Vector3d(3.0, 0.0, 0.02),
      Eigen::Vector3d(1.9, 1.25, 0.04),
      Eigen::Vector3d(0.20, 0.52, 0.20)));

  world->addSimpleFrame(createLineMarker(
      "ball_rotation_cue",
      Eigen::Vector3d(-3.55, 0.0, 2.1),
      Eigen::Vector3d(-2.45, 0.0, 1.5),
      4.0f,
      Eigen::Vector3d(1.0, 0.80, 0.20)));
  world->addSimpleFrame(createLineMarker(
      "weld_lock_cue_a",
      Eigen::Vector3d(2.45, -0.35, 1.55),
      Eigen::Vector3d(3.55, 0.35, 0.75),
      4.0f,
      Eigen::Vector3d(0.95, 0.95, 0.95)));
  world->addSimpleFrame(createLineMarker(
      "weld_lock_cue_b",
      Eigen::Vector3d(2.45, 0.35, 1.55),
      Eigen::Vector3d(3.55, -0.35, 0.75),
      4.0f,
      Eigen::Vector3d(0.95, 0.95, 0.95)));
}

dart::dynamics::SimpleFramePtr createAxisMarker(
    const std::string& name,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Vector3d& color)
{
  auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(), name);
  auto arrow = std::make_shared<dart::dynamics::ArrowShape>(
      start,
      end,
      dart::dynamics::ArrowShape::Properties(0.025, 2.2, 0.18, 0.08, 0.18),
      Eigen::Vector4d(color[0], color[1], color[2], 1.0));
  frame->setShape(arrow);
  frame->createVisualAspect();
  frame->getVisualAspect()->setColor(color);
  return frame;
}

void addBallJointDemo(const dart::simulation::WorldPtr& world)
{
  const Eigen::Vector3d anchor(-3.0, 0.0, 1.8);
  DemoBody body = createFreeBox(
      "ball_body",
      makeTransform(anchor + Eigen::Vector3d(0.65, 0.0, -0.45)),
      Eigen::Vector3d(0.75, 0.25, 0.25),
      Eigen::Vector3d(0.85, 0.25, 0.20));
  setFreeJointVelocity(
      body.joint, Eigen::Vector3d(0.0, 1.4, 0.0), Eigen::Vector3d::Zero());
  world->addSkeleton(body.skeleton);
  world->addSimpleFrame(
      createSphereMarker("ball_anchor", anchor, 0.08, Eigen::Vector3d::Ones()));
  world->addSimpleFrame(createLineMarker(
      "ball_anchor_line",
      anchor + Eigen::Vector3d(0.0, 0.0, -0.45),
      anchor + Eigen::Vector3d(0.0, 0.0, 0.45),
      2.0f,
      Eigen::Vector3d(0.75, 0.75, 0.75)));

  auto constraint = std::make_shared<dart::constraint::BallJointConstraint>(
      body.body, anchor);
  world->getConstraintSolver()->addConstraint(constraint);
}

void addCylindricalJointDemo(const dart::simulation::WorldPtr& world)
{
  const Eigen::Vector3d axisPoint(0.0, 0.0, 1.15);
  const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  DemoBody body = createFreeBox(
      "cylindrical_body",
      makeTransform(axisPoint),
      Eigen::Vector3d(0.28, 0.65, 0.25),
      Eigen::Vector3d(0.20, 0.55, 0.95),
      Eigen::Vector3d(0.35, 0.0, 0.0));
  setFreeJointVelocity(
      body.joint,
      Eigen::Vector3d(0.0, 0.0, 4.0),
      Eigen::Vector3d(0.0, 0.0, 1.2));
  world->addSkeleton(body.skeleton);
  world->addSimpleFrame(createLineMarker(
      "cylindrical_axis",
      axisPoint - 1.15 * axis,
      axisPoint + 1.65 * axis,
      4.0f,
      Eigen::Vector3d(0.20, 0.85, 1.0)));
  world->addSimpleFrame(createAxisMarker(
      "cylindrical_axis_arrow",
      axisPoint - 1.15 * axis,
      axisPoint + 1.65 * axis,
      Eigen::Vector3d(0.20, 0.85, 1.0)));

  auto constraint
      = std::make_shared<dart::constraint::CylindricalJointConstraint>(
          body.body, axisPoint, axis);
  world->getConstraintSolver()->addConstraint(constraint);
}

void addWeldJointDemo(const dart::simulation::WorldPtr& world)
{
  const Eigen::Vector3d weldPosition(3.0, 0.0, 1.15);
  DemoBody body = createFreeBox(
      "weld_body",
      makeTransform(weldPosition),
      Eigen::Vector3d(0.70, 0.32, 0.32),
      Eigen::Vector3d(0.40, 0.80, 0.35));
  setFreeJointVelocity(
      body.joint,
      Eigen::Vector3d(0.7, 0.0, 0.9),
      Eigen::Vector3d(0.6, 0.0, 0.4));
  world->addSkeleton(body.skeleton);
  world->addSimpleFrame(createSphereMarker(
      "weld_reference", weldPosition, 0.08, Eigen::Vector3d(0.40, 0.80, 0.35)));
  world->addSimpleFrame(createLineMarker(
      "weld_reference_line",
      weldPosition - Eigen::Vector3d(0.45, 0.0, 0.0),
      weldPosition + Eigen::Vector3d(0.45, 0.0, 0.0),
      3.0f,
      Eigen::Vector3d(0.40, 0.80, 0.35)));

  auto constraint
      = std::make_shared<dart::constraint::WeldJointConstraint>(body.body);
  world->getConstraintSolver()->addConstraint(constraint);
}

dart::simulation::WorldPtr createWorld()
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -4.0));
  world->setTimeStep(1e-3);

  addSceneGuides(world);
  addBallJointDemo(world);
  addCylindricalJointDemo(world);
  addWeldJointDemo(world);
  return world;
}

class DynamicJointWidget : public dart::gui::osg::ImGuiWidget
{
public:
  DynamicJointWidget(
      dart::gui::osg::ImGuiViewer* viewer,
      dart::simulation::WorldPtr world,
      double guiScale)
    : mViewer(viewer),
      mWorld(std::move(world)),
      mGuiScale(static_cast<float>(guiScale))
  {
    // Do nothing.
  }

  void render() override
  {
    const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    const float margin = 10.0f * mGuiScale;
    const ImVec2 windowSize = computeWindowSize(displaySize, margin);
    ImGui::SetNextWindowPos(
        ImVec2(std::max(margin, displaySize.x - windowSize.x - margin), margin),
        ImGuiCond_Always);
    ImGui::SetNextWindowSize(windowSize, ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.80f);
    if (!ImGui::Begin(
            "Dynamic Joint Constraints",
            nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar
                | ImGuiWindowFlags_NoSavedSettings)) {
      ImGui::End();
      return;
    }

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

    int simulationState = mViewer->isSimulating() ? 0 : 1;
    if (ImGui::RadioButton("Play", &simulationState, 0)
        && !mViewer->isSimulating())
      mViewer->simulate(true);
    ImGui::SameLine();
    if (ImGui::RadioButton("Pause", &simulationState, 1)
        && mViewer->isSimulating())
      mViewer->simulate(false);

    ImGui::Text("Time: %.2f", mWorld->getTime());
    ImGui::Separator();
    ImGui::TextWrapped("Ball: fixed anchor / free rotation");
    ImGui::TextWrapped("Cylindrical: fixed axis / slide + spin");
    ImGui::TextWrapped("Weld: fixed pose");
    ImGui::Separator();
    ImGui::Text(
        "Constraints: %u",
        static_cast<unsigned>(
            mWorld->getConstraintSolver()->getNumConstraints()));

    ImGui::End();
  }

private:
  ImVec2 computeWindowSize(const ImVec2& displaySize, float margin) const
  {
    const ImVec2 preferred(kPanelWidth * mGuiScale, kPanelHeight * mGuiScale);
    if (displaySize.x <= 0.0f || displaySize.y <= 0.0f)
      return preferred;

    const float maxWidth = std::max(1.0f, displaySize.x - 2.0f * margin);
    const float maxHeight = std::max(1.0f, displaySize.y - 2.0f * margin);
    return ImVec2(
        std::min(preferred.x, maxWidth), std::min(preferred.y, maxHeight));
  }

  osg::ref_ptr<dart::gui::osg::ImGuiViewer> mViewer;
  dart::simulation::WorldPtr mWorld;
  float mGuiScale;
};

} // namespace

int main(int argc, char* argv[])
{
  const dart::gui::osg::GuiScaleOptions options
      = dart::gui::osg::parseGuiScaleOptions(argc, argv, &std::cerr);
  if (options.showHelp) {
    dart::gui::osg::printGuiScaleUsage(std::cout, argv[0]);
    return 0;
  }

  dart::simulation::WorldPtr world = createWorld();

  osg::ref_ptr<dart::gui::osg::WorldNode> node
      = new dart::gui::osg::WorldNode(world);

  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
      = new dart::gui::osg::ImGuiViewer();
  viewer->addWorldNode(node);
  viewer->getImGuiHandler()->setGuiScale(options.scale);
  viewer->getRootGroup()->addChild(createSceneLabels(options.scale));
  viewer->getImGuiHandler()->addWidget(
      std::make_shared<DynamicJointWidget>(viewer, world, options.scale));

  viewer->addInstructionText(
      "\nThe scene shows Ball, Cylindrical, and Weld dynamic constraints.\n");
  viewer->addInstructionText(
      "The cylindrical body is constrained to the center rail while it slides "
      "and spins.\n");
  viewer->addInstructionText(
      "Use --gui-scale <value> to scale the ImGui panel and viewer window.\n");
  std::cout << viewer->getInstructions() << std::endl;

  viewer->setUpViewInWindow(
      0,
      0,
      dart::gui::osg::scaleWindowExtent(kViewerWidth, options.scale),
      dart::gui::osg::scaleWindowExtent(kViewerHeight, options.scale));
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(5.0, -7.0, 4.0),
      ::osg::Vec3(0.0, 0.0, 0.9),
      ::osg::Vec3(0.0, 0.0, 1.0));
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  viewer->simulate(true);
  viewer->run();
}
