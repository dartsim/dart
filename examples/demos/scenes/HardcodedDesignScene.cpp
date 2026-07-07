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

// Ported from examples/hardcoded_design: a 3-dof revolute leg built entirely
// from code (no model file), posed directly from the keyboard (keys 1-3 step
// a dof; '-' flips the step direction).
//
// Deviation from the original: the original never runs physics (it wraps a
// plain, non-RealTime WorldNode whose `mSimulating` flag is never turned on,
// so this is a purely kinematic pose demo). dart-demos always drives the
// active scene's world through a RealTimeWorldNode, so pressing the host's
// Play control here would apply gravity to a skeleton with no ground and no
// PD control, causing it to fall away from the pose keys set. Gravity is
// disabled for this scene's world to preserve the original's kinematic-only
// behavior regardless of Play/Pause state.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <osg/PolygonMode>

#include <algorithm>
#include <memory>

#include <cmath>

namespace dart_demos {

namespace {

//==============================================================================
dart::dynamics::SkeletonPtr createSkeleton()
{
  using dart::dynamics::BodyNode;
  using dart::dynamics::BoxShape;
  using dart::dynamics::RevoluteJoint;
  using dart::dynamics::Skeleton;

  auto legSkel = Skeleton::create();

  const double mass = 1.0;

  // BodyNode 1: Left Hip Yaw (LHY)
  BodyNode::Properties body;
  body.mName = "LHY";
  dart::dynamics::ShapePtr shape(new BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0)));
  body.mInertia.setMass(mass);

  RevoluteJoint::Properties joint;
  joint.mName = "LHY";
  joint.mAxis = Eigen::Vector3d(0.0, 0.0, 1.0);
  joint.mPositionLowerLimits[0] = -dart::math::constantsd::pi();
  joint.mPositionUpperLimits[0] = dart::math::constantsd::pi();

  auto pair = legSkel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, joint, body);
  pair.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  BodyNode* parent = pair.second;

  // BodyNode 2: Left Hip Roll (LHR), parent LHY
  body = BodyNode::Properties();
  body.mName = "LHR";
  shape
      = dart::dynamics::ShapePtr(new BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0)));

  joint.mName = "LHR";
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 0.5);

  auto pair1
      = legSkel->createJointAndBodyNodePair<RevoluteJoint>(parent, joint, body);
  pair1.first->setAxis(Eigen::Vector3d(1.0, 0.0, 0.0));
  auto shapeNode1 = pair1.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode1->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));
  pair1.second->setLocalCOM(shapeNode1->getRelativeTranslation());
  pair1.second->setMass(mass);

  // BodyNode 3: Left Hip Pitch (LHP), parent LHR
  body = BodyNode::Properties();
  body.mName = "LHP";
  shape
      = dart::dynamics::ShapePtr(new BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0)));

  joint.mName = "LHP";
  joint.mAxis = Eigen::Vector3d(0.0, 1.0, 0.0);
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 1.0);

  auto pair2 = legSkel->createJointAndBodyNodePair<RevoluteJoint>(
      legSkel->getBodyNode(1), joint, body);
  auto shapeNode2 = pair2.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode2->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));
  pair2.second->setLocalCOM(shapeNode2->getRelativeTranslation());
  pair2.second->setMass(mass);

  return legSkel;
}

//==============================================================================
struct HardcodedDesignState
{
  dart::dynamics::SkeletonPtr skeleton;
  bool inverse = false;
  double step = 0.1;
};

//==============================================================================
void stepDof(HardcodedDesignState& state, std::size_t dofIdx)
{
  if (dofIdx >= state.skeleton->getNumDofs())
    return;

  Eigen::VectorXd pose = state.skeleton->getPositions();
  pose(dofIdx) += state.inverse ? -state.step : state.step;
  state.skeleton->setPositions(pose);
}

} // namespace

//==============================================================================
DemoScene makeHardcodedDesignScene()
{
  DemoScene scene;
  scene.id = "hardcoded_design";
  scene.title = "Hardcoded Design";
  scene.category = "Constraints & Joints";
  scene.summary = "A hand-built revolute chain driven from the keyboard.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();
    world->setGravity(Eigen::Vector3d::Zero());

    auto skeleton = createSkeleton();
    world->addSkeleton(skeleton);

    auto state = std::make_shared<HardcodedDesignState>();
    state->skeleton = skeleton;

    DemoSceneSetup setup;
    setup.world = world;
    // The original never enabled shadows, and this scene renders as wireframe
    // (see onActivate below); shadow casters over line geometry are a pointless
    // extra pass, so keep the shadow technique off to match the original.
    setup.enableShadows = false;
    // The three stacked 1 m links extend from roughly z = -0.5 up to z ~ 2.5,
    // so pull the camera well back and aim at the leg's mid height (z ~ 1) to
    // frame the whole ~3 m limb with margin under the headless 30-degree
    // vertical FOV (the original's tighter 2,2,2 -> origin view clipped the
    // upper links).
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(6.5, 6.5, 3.0),
        ::osg::Vec3d(0.0, 0.0, 1.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    // Match the original's wireframe rendering style. osg::Node::getStateSet()
    // (unlike getOrCreateStateSet()) may return null if no StateSet has been
    // created yet, so use the creating accessor to avoid a null dereference.
    setup.onActivate = [](DemoHostContext& ctx) {
      if (auto* node = ctx.worldNode()) {
        node->getOrCreateStateSet()->setAttributeAndModes(
            new ::osg::PolygonMode(
                ::osg::PolygonMode::FRONT_AND_BACK, ::osg::PolygonMode::LINE),
            ::osg::StateAttribute::ON | ::osg::StateAttribute::OVERRIDE);
      }
    };

    setup.keyActions.push_back(KeyAction{'-', "Invert direction", [state] {
                                           state->inverse = !state->inverse;
                                         }});
    setup.keyActions.push_back(KeyAction{'1', "Move LHY", [state] {
                                           stepDof(*state, 0);
                                         }});
    setup.keyActions.push_back(KeyAction{'2', "Move LHR", [state] {
                                           stepDof(*state, 1);
                                         }});
    setup.keyActions.push_back(KeyAction{'3', "Move LHP", [state] {
                                           stepDof(*state, 2);
                                         }});

    setup.renderPanel = [state] {
      ImGui::Text("Direction: %s", state->inverse ? "negative" : "positive");

      auto step = static_cast<float>(state->step);
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
      if (ImGui::SliderFloat(
              "Step size",
              &step,
              0.01f,
              0.5f,
              "%.3f",
              ImGuiSliderFlags_AlwaysClamp)
          && std::isfinite(step))
        state->step = std::clamp(step, 0.01f, 0.5f);

      const Eigen::VectorXd pose = state->skeleton->getPositions();
      ImGui::Text(
          "Pose: [%.2f, %.2f, %.2f]",
          pose.size() > 0 ? pose(0) : 0.0,
          pose.size() > 1 ? pose(1) : 0.0,
          pose.size() > 2 ? pose(2) : 0.0);
      ImGui::TextWrapped(
          "This is a purely kinematic pose demo (gravity disabled): keys "
          "1/2/3 step the LHY/LHR/LHP joints, '-' flips the direction.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
