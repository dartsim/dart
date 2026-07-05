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

// Ported from examples/fetch: a Fetch robot parsed from an MJCF pick-and-place
// scene, whose end effector is welded (via an MJCF equality constraint) to an
// invisible mocap body that a draggable InteractiveFrame drives kinematically
// every step.
//
// Deviations from the original: the original unconditionally requires Bullet
// (`#include <dart/collision/bullet/bullet.hpp>` +
// `collision::BulletCollisionDetector::create()`, and its CMakeLists.txt
// REQUIREs the collision-bullet component). This port instead uses the same
// graceful, string-keyed CollisionDetector factory check already established
// by B1's RigidShapesScene/AddDeleteSkelsScene for other originally
// Bullet-only examples: `canCreate("bullet")` picks up dart-collision-bullet
// when the host links it (which examples/demos/CMakeLists.txt already does
// whenever the backend is built) and otherwise falls back to DART's default
// detector with a note logged to Diagnostics, rather than failing to compile
// or throwing outright -- this needs no new optional-dependency plumbing in
// CMakeLists.txt (unlike TinyDNN/Python/octomap) since the fallback is a
// runtime check, not a missing symbol. The original's `DART_ASSERT` calls
// (no-ops in release builds, so a missing skeleton/constraint would
// dereference a nullptr silently) are replaced with `throw
// std::runtime_error` guards, matching every other scene factory's
// "throw on missing assets" convention. The Fetch-specific ImGui panel
// (menu bar, Play/Pause radio, Help section) is dropped as redundant with
// host chrome, keeping only the descriptive text.

#include "Scenes.hpp"

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <memory>
#include <stdexcept>

namespace dart_demos {

namespace {

//==============================================================================
/// Per-instance state captured by this scene's preStep lambda: drives the
/// mocap body to the interactive frame's transform every step, exactly as
/// the original's FetchWorldNode::customPreStep did.
struct FetchState
{
  dart::dynamics::BodyNode* mocap = nullptr;
  dart::dynamics::Frame* interactiveFrame = nullptr;
};

} // namespace

//==============================================================================
DemoScene makeFetchScene()
{
  DemoScene scene;
  scene.id = "fetch";
  scene.title = "Fetch";
  scene.category = "Robots";
  scene.summary = "A Fetch robot's end effector follows an interactive target.";

  scene.factory = [] {
    auto world = dart::utils::MjcfParser::readWorld(
        "dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml");
    if (!world)
      throw std::runtime_error(
          "failed to load dart://sample/mjcf/openai/robotics/fetch/"
          "pick_and_place.xml");

    // The original unconditionally requires Bullet; use the same graceful,
    // string-keyed factory check as RigidShapesScene/AddDeleteSkelsScene
    // instead (see the file comment).
    if (dart::collision::CollisionDetector::getFactory()->canCreate("bullet")) {
      world->getConstraintSolver()->setCollisionDetector(
          dart::collision::CollisionDetector::getFactory()->create("bullet"));
    }

    auto robot = world->getSkeleton("robot0:base_link");
    if (!robot)
      throw std::runtime_error(
          "pick_and_place.xml: missing skeleton 'robot0:base_link'");
    robot->getJoint(0)->setActuatorType(
        dart::dynamics::Joint::ActuatorType::LOCKED);
    robot->eachDof([](dart::dynamics::DegreeOfFreedom* dof) {
      dof->setSpringStiffness(1e+3);
    });
    auto* torsoLiftJoint = robot->getJoint("robot0:torso_lift_joint");
    if (!torsoLiftJoint)
      throw std::runtime_error(
          "pick_and_place.xml: missing joint 'robot0:torso_lift_joint'");
    torsoLiftJoint->setSpringStiffness(0, 1e+7);

    if (robot->getNumDofs() < 13)
      throw std::runtime_error(
          "pick_and_place.xml: expected >= 13 dofs on 'robot0:base_link'");
    robot->setPosition(0, 0.405);
    robot->setPosition(1, 0.480);
    robot->setPosition(2, 0.000);
    robot->setPosition(6, 0.01);
    robot->setPosition(7, -0.73);
    robot->setPosition(8, 0.00);
    robot->setPosition(9, 1.64);
    robot->setPosition(10, 0.0);
    robot->setPosition(11, 0.66);
    robot->setPosition(12, 0.01);

    auto object = world->getSkeleton("object0");
    if (!object)
      throw std::runtime_error(
          "pick_and_place.xml: missing skeleton 'object0'");
    if (object->getNumDofs() < 6)
      throw std::runtime_error(
          "pick_and_place.xml: expected >= 6 dofs on 'object0'");
    object->setPosition(3, 1.25);
    object->setPosition(4, 0.53);
    object->setPosition(5, 0.40);

    auto mocap = world->getSkeleton("robot0:mocap");
    if (!mocap)
      throw std::runtime_error(
          "pick_and_place.xml: missing skeleton 'robot0:mocap'");

    auto weldJointConstraint
        = std::dynamic_pointer_cast<dart::constraint::WeldJointConstraint>(
            world->getConstraintSolver()->getConstraint(0));
    if (!weldJointConstraint)
      throw std::runtime_error(
          "pick_and_place.xml: expected a WeldJointConstraint at constraint "
          "index 0 (mocap <-> end effector)");
    weldJointConstraint->setRelativeTransform(Eigen::Isometry3d::Identity());

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(1.3, 0.75, 0.50);
    tf.linear()
        = Eigen::AngleAxisd(
              dart::math::constantsd::pi() / 2, Eigen::Vector3d::UnitY())
              .toRotationMatrix();
    auto frame = std::make_shared<dart::gui::osg::InteractiveFrame>(
        dart::dynamics::Frame::World(), "interactive frame", tf, 0.2);
    world->addSimpleFrame(frame);

    auto state = std::make_shared<FetchState>();
    state->mocap = mocap->getRootBodyNode();
    state->interactiveFrame = frame.get();

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(4.0, 4.0, 2.5),
        ::osg::Vec3d(0.1, -0.3, 0.3),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.preStep = [state] {
      if (!state->mocap || !state->interactiveFrame)
        return;
      state->mocap->getParentJoint()->setTransformFromParentBodyNode(
          state->interactiveFrame->getTransform());
    };

    setup.onActivate = [frame](DemoHostContext& ctx) {
      auto* viewer = ctx.viewer();

      auto* grid = new dart::gui::osg::GridVisual();
      grid->setOffset(Eigen::Vector3d(1.3, 0.75, 0));
      ctx.addAttachment(grid);

      if (auto* dnd = viewer->enableDragAndDrop(frame.get()))
        ctx.addTeardown([viewer, dnd] { viewer->disableDragAndDrop(dnd); });
    };

    setup.renderPanel = [] {
      ImGui::TextWrapped(
          "The Fetch robot's whole-body motion emerges from joint springs "
          "plus a welded end effector that follows the invisible mocap "
          "target -- drag the gizmo (crossed by the transparent green grid "
          "bars) to move it while the simulation is playing.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
