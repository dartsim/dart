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

// Ported from examples/dynamic_joint_constraints: three free-flying boxes,
// each locked to a dynamic joint constraint added once to the
// ConstraintSolver -- Ball (fixed anchor point, free rotation), Cylindrical
// (fixed axis, slide + spin), and Weld (fixed pose, no free DOF).
//
// Deviations from the original: the Play/Pause radio buttons and the "Time:"
// readout are dropped from the scene panel -- the host's Simulation toolbar
// and Diagnostics panel already own both, generically, for every scene (the
// same precedent SleepingScene documents). The Menu bar (Exit / Help > About
// DART) is dropped too: no other ported scene recreates that chrome, since it
// duplicates functionality the host itself doesn't yet expose per-scene. The
// on-screen osgText labels (title + BALL/CYLINDRICAL/WELD captions) are kept.
// Because dart-demos is a single long-lived window where scenes come and go,
// the label group is added to the viewer's root group from onActivate and
// explicitly removed on teardown (the original added it once in main() for
// the process lifetime, since it never tore anything down); the group is
// still sized from the gui scale once at construction, matching the
// original's own one-shot (non-reactive) sizing.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <osg/Geode>
#include <osg/Group>
#include <osg/StateSet>
#include <osgText/Text>

#include <memory>

namespace dart_demos {

namespace {

Eigen::Isometry3d makeTransform(const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  return transform;
}

struct DemoBody
{
  dart::dynamics::SkeletonPtr skeleton;
  dart::dynamics::FreeJoint* joint;
  dart::dynamics::BodyNode* body;
};

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
  result.body->setMass(1.0);

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

::osg::ref_ptr<::osg::Geode> createTextLabel(
    const std::string& text,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color,
    float characterSize)
{
  ::osg::ref_ptr<::osgText::Text> label = new ::osgText::Text();
  label->setText(text);
  label->setPosition(::osg::Vec3(position.x(), position.y(), position.z()));
  label->setColor(::osg::Vec4(color.x(), color.y(), color.z(), 1.0));
  label->setCharacterSizeMode(::osgText::TextBase::SCREEN_COORDS);
  label->setCharacterSize(characterSize);
  label->setAxisAlignment(::osgText::TextBase::SCREEN);
  label->setAlignment(::osgText::TextBase::CENTER_CENTER);
  label->setBackdropType(::osgText::Text::OUTLINE);
  label->setBackdropColor(::osg::Vec4(1.0, 1.0, 1.0, 1.0));
  label->setEnableDepthWrites(false);

  ::osg::ref_ptr<::osg::Geode> geode = new ::osg::Geode();
  geode->addDrawable(label);
  geode->getOrCreateStateSet()->setMode(
      GL_DEPTH_TEST, ::osg::StateAttribute::OFF);
  return geode;
}

::osg::ref_ptr<::osg::Group> createSceneLabels(double guiScale)
{
  const float baseSize = static_cast<float>(18.0 * guiScale);
  const float titleSize = static_cast<float>(22.0 * guiScale);

  ::osg::ref_ptr<::osg::Group> labels = new ::osg::Group();
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

} // namespace

//==============================================================================
DemoScene makeDynamicJointConstraintsScene()
{
  DemoScene scene;
  scene.id = "dynamic_joint_constraints";
  scene.title = "Dynamic Joint Constraints";
  scene.category = "Constraints & Joints";
  scene.summary
      = "Ball, cylindrical, and weld dynamic joint constraints on three "
        "free-flying bodies.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -4.0));
    world->setTimeStep(1e-3);

    addSceneGuides(world);
    addBallJointDemo(world);
    addCylindricalJointDemo(world);
    addWeldJointDemo(world);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(5.0, -7.0, 4.0),
        ::osg::Vec3d(0.0, 0.0, 0.9),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.onActivate = [](DemoHostContext& ctx) {
      auto* viewer = ctx.viewer();
      const double guiScale = viewer->getImGuiHandler()->getGuiScale();
      ::osg::ref_ptr<::osg::Group> labels = createSceneLabels(guiScale);
      viewer->getRootGroup()->addChild(labels);
      ctx.addTeardown(
          [viewer, labels] { viewer->getRootGroup()->removeChild(labels); });
    };

    setup.renderPanel = [world] {
      ImGui::TextWrapped("Ball: fixed anchor / free rotation");
      ImGui::TextWrapped("Cylindrical: fixed axis / slide + spin");
      ImGui::TextWrapped("Weld: fixed pose");
      ImGui::Separator();
      ImGui::Text(
          "Constraints: %u",
          static_cast<unsigned>(
              world->getConstraintSolver()->getNumConstraints()));
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
