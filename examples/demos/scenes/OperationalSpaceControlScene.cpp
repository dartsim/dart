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

// Ported from examples/operational_space_control: a KR5 arm chasing a
// draggable red-ball target with a damped-pseudoinverse task-space
// controller, plus held-key (1/2/3) axis/plane drag constraints and an 's'
// shadow toggle.
//
// Deviations from the original: the robot and ground are already Z-up in the
// original (root joint reset to Identity, ground explicitly rotated), so no
// ZUp.hpp reorientation is applied here, unlike the other B3 scenes built on
// Y-up .skel worlds. The held-key axis-constraint feature needs both
// KEYDOWN and KEYUP, which DemoScene's KeyAction (key-down only, mirrored as
// a panel button) cannot express; this is ported via a small
// osgGA::GUIEventHandler registered through DemoHostContext::addEventHandler,
// exactly like the original's ConstraintEventHandler, so the interaction is
// unchanged (hold 1/2/3, alone or combined, to constrain the drag to a line
// or plane). The original's setNumStepsPerCycle(10) (a fixed number of
// physics steps per rendered frame on a plain, non-real-time WorldNode) has
// no equivalent in this host, which paces every scene through
// RealTimeWorldNode's wall-clock-driven Target RTF slider instead; that
// slider supersedes the fixed-10 setting; this is a dropped tunable, noted
// in the panel text. DnD is registered in onActivate (after the world node is
// attached, matching the original's setupViewer() timing) rather than via a
// public member set later, avoiding the original's documented
// construct-before-attach portRisk.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <memory>
#include <stdexcept>

namespace dart_demos {

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::SkeletonPtr;

//==============================================================================
/// Per-instance state captured by this scene's preStep/onActivate lambdas and
/// the held-key constraint handler.
struct OscState
{
  SkeletonPtr robot;
  BodyNode* endEffector = nullptr;
  dart::dynamics::SimpleFramePtr target;

  Eigen::Vector3d offset = Eigen::Vector3d(0.05, 0, 0);
  Eigen::Matrix3d kp = Eigen::Matrix3d::Zero();
  Eigen::MatrixXd kd;

  dart::gui::osg::DragAndDrop* dnd = nullptr;
  bool constrained[3] = {false, false, false};

  // Set from onActivate (see the file comment on why key actions -- built at
  // factory time, before the world node exists -- must read these lazily
  // instead of capturing them directly).
  dart::gui::osg::WorldNode* worldNode = nullptr;
  dart::gui::osg::Viewer* viewer = nullptr;
};

//==============================================================================
void toggleShadows(OscState& state)
{
  if (!state.worldNode || !state.viewer)
    return;
  if (state.worldNode->isShadowed())
    state.worldNode->setShadowTechnique(nullptr);
  else
    state.worldNode->setShadowTechnique(
        dart::gui::osg::WorldNode::createDefaultShadowTechnique(state.viewer));
}

//==============================================================================
void applyDndConstraint(OscState& state)
{
  if (!state.dnd)
    return;

  std::size_t constraintDofs = 0;
  for (bool c : state.constrained)
    if (c)
      ++constraintDofs;

  if (constraintDofs == 0 || constraintDofs == 3) {
    state.dnd->unconstrain();
  } else if (constraintDofs == 1) {
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < 3; ++i)
      if (state.constrained[i])
        v[i] = 1.0;
    state.dnd->constrainToLine(v);
  } else {
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < 3; ++i)
      if (!state.constrained[i])
        v[i] = 1.0;
    state.dnd->constrainToPlane(v);
  }
}

//==============================================================================
/// Ported from the original's ConstraintEventHandler: holds keys 1/2/3
/// (KEYDOWN sets, KEYUP clears) constrain the target's drag to the
/// corresponding world axis/axes.
class OscConstraintHandler : public ::osgGA::GUIEventHandler
{
public:
  explicit OscConstraintHandler(std::shared_ptr<OscState> state)
    : mState(std::move(state))
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    int axis = -1;
    switch (ea.getKey()) {
      case '1':
        axis = 0;
        break;
      case '2':
        axis = 1;
        break;
      case '3':
        axis = 2;
        break;
      default:
        return false;
    }

    if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN) {
      mState->constrained[axis] = true;
    } else if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYUP) {
      mState->constrained[axis] = false;
    } else {
      return false;
    }

    applyDndConstraint(*mState);
    return true;
  }

private:
  std::shared_ptr<OscState> mState;
};

} // namespace

//==============================================================================
DemoScene makeOperationalSpaceControlScene()
{
  DemoScene scene;
  scene.id = "operational_space_control";
  scene.title = "Operational Space Control";
  scene.category = "Control & IK";
  scene.summary = "KR5 arm chasing a draggable target with task-space control.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();

    dart::utils::DartLoader loader;
    auto robot
        = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
    if (!robot)
      throw std::runtime_error(
          "failed to load dart://sample/urdf/KR5/KR5 sixx R650.urdf");
    world->addSkeleton(robot);
    robot->getJoint(0)->setTransformFromParentBodyNode(
        Eigen::Isometry3d::Identity());

    auto ground = loader.parseSkeleton("dart://sample/urdf/KR5/ground.urdf");
    if (!ground)
      throw std::runtime_error(
          "failed to load dart://sample/urdf/KR5/ground.urdf");
    world->addSkeleton(ground);
    Eigen::Isometry3d groundTf
        = ground->getJoint(0)->getTransformFromParentBodyNode();
    groundTf.pretranslate(Eigen::Vector3d(0, 0, 0.5));
    groundTf.rotate(Eigen::AngleAxisd(
        dart::math::constantsd::pi() / 2, Eigen::Vector3d(1, 0, 0)));
    ground->getJoint(0)->setTransformFromParentBodyNode(groundTf);

    auto state = std::make_shared<OscState>();
    state->robot = robot;
    state->endEffector = robot->getBodyNode(robot->getNumBodyNodes() - 1);

    const std::size_t dofs = state->endEffector->getNumDependentGenCoords();
    for (std::size_t i = 0; i < 3; ++i)
      state->kp(i, i) = 50.0;
    state->kd = Eigen::MatrixXd::Zero(dofs, dofs);
    for (std::size_t i = 0; i < dofs; ++i)
      state->kd(i, i) = 5.0;

    robot->eachJoint([](dart::dynamics::Joint* joint) {
      joint->setLimitEnforcement(false);
      joint->setDampingCoefficient(0, 0.5);
    });

    Eigen::Isometry3d tf = state->endEffector->getWorldTransform();
    tf.pretranslate(state->offset);
    state->target = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "target", tf);
    state->target->setShape(
        std::make_shared<dart::dynamics::SphereShape>(0.025));
    state->target->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0, 0));
    world->addSimpleFrame(state->target);

    state->offset
        = state->endEffector->getWorldTransform().rotation().transpose()
          * state->offset;

    DemoSceneSetup setup;
    setup.world = world;
    // Match the original's startup: it begins unshadowed (a plain WorldNode)
    // and only creates a shadow technique on the first 's' press, which the
    // 's' key action below toggles.
    setup.enableShadows = false;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(2.57, 3.14, 1.64),
        ::osg::Vec3d(0.00, 0.00, 0.00),
        ::osg::Vec3d(-0.24, -0.25, 0.94)};

    setup.preStep = [state] {
      auto* robot = state->robot.get();
      auto* ee = state->endEffector;

      Eigen::MatrixXd M = robot->getMassMatrix();

      dart::math::LinearJacobian J = ee->getLinearJacobian(state->offset);
      Eigen::MatrixXd pinvJ
          = J.transpose()
            * (J * J.transpose() + 0.0025 * Eigen::Matrix3d::Identity())
                  .inverse();

      dart::math::LinearJacobian dJ = ee->getLinearJacobianDeriv(state->offset);
      Eigen::MatrixXd pinvDJ
          = dJ.transpose()
            * (dJ * dJ.transpose() + 0.0025 * Eigen::Matrix3d::Identity())
                  .inverse();

      Eigen::Vector3d e = state->target->getWorldTransform().translation()
                          - ee->getWorldTransform() * state->offset;
      Eigen::Vector3d de = -ee->getLinearVelocity(state->offset);
      Eigen::VectorXd Cg = robot->getCoriolisAndGravityForces();

      Eigen::VectorXd forces
          = M * (pinvJ * state->kp * de + pinvDJ * state->kp * e) + Cg
            + state->kd * pinvJ * state->kp * e;
      robot->setForces(forces);
    };

    setup.onActivate = [state](DemoHostContext& ctx) {
      state->worldNode = ctx.worldNode();
      state->viewer = ctx.viewer();

      auto* viewer = ctx.viewer();
      state->dnd = viewer->enableDragAndDrop(state->target.get());
      if (state->dnd) {
        state->dnd->setObstructable(false);
        ctx.addTeardown(
            [viewer, dnd = state->dnd] { viewer->disableDragAndDrop(dnd); });
      }

      ctx.addEventHandler(new OscConstraintHandler(state));
    };

    // Keys 1/2/3 (held, not pressed) are handled directly by
    // OscConstraintHandler registered in onActivate below -- see the file
    // comment on why they cannot be expressed as ordinary (key-down-only)
    // KeyActions.
    setup.keyActions.push_back(KeyAction{'s', "Toggle shadows", [state] {
                                           toggleShadows(*state);
                                         }});

    setup.renderPanel = [state] {
      ImGui::Text(
          "Drag constraint: %s%s%s%s",
          state->constrained[0] ? "x " : "",
          state->constrained[1] ? "y " : "",
          state->constrained[2] ? "z " : "",
          (!state->constrained[0] && !state->constrained[1]
           && !state->constrained[2])
              ? "(none)"
              : "");
      ImGui::TextWrapped(
          "Drag the red ball to move the operational-space controller's "
          "target. Hold 1/2/3 (alone or combined) to constrain the drag to "
          "an axis or plane. The original's fixed 10-steps-per-render-cycle "
          "pacing is superseded by the host's Target RTF slider.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
