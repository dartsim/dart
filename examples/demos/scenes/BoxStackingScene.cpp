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

// Ported from examples/box_stacking: 5 boxes stacked on a floor, comparing the
// Dantzig and PGS boxed-LCP constraint solvers.
//
// Deviation from the original: the original's ImGui panel offered three LCP
// radio buttons, but two of them were both labeled "Dantzig" and both built a
// Dantzig solver (a leftover "SI" option was commented out without updating
// the label/index above it), so only two solvers were ever actually
// reachable. This port exposes just those two real choices (Dantzig, PGS).
// The original's gravity and headlights toggles are dropped: dart-demos'
// host-level Simulation toolbar already exposes a gravity toggle, and
// headlights are host chrome, not scene-specific state. The original's custom
// event handler (q/Q and Left/Right printing "pressed"/"released" lines to
// stdout) is also dropped: it only wrote to the console and changed no
// simulation state, and dart-demos has no user-facing console.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <memory>

namespace dart_demos {

namespace {

//==============================================================================
dart::dynamics::SkeletonPtr createBox(const Eigen::Vector3d& position)
{
  using namespace dart::dynamics;

  auto boxSkel = Skeleton::create("box");
  auto boxBody = boxSkel->createJointAndBodyNodePair<FreeJoint>(nullptr).second;

  constexpr double boxWidth = 1.0;
  constexpr double boxDepth = 1.0;
  constexpr double boxHeight = 0.5;
  auto boxShape = std::make_shared<BoxShape>(
      Eigen::Vector3d(boxWidth, boxDepth, boxHeight));
  auto* shapeNode = boxBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(
      dart::math::Random::uniform<Eigen::Vector3d>(0.0, 1.0));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  boxBody->getParentJoint()->setTransformFromParentBodyNode(tf);

  return boxSkel;
}

//==============================================================================
std::vector<dart::dynamics::SkeletonPtr> createBoxStack(
    std::size_t numBoxes, double heightFromGround = 0.5)
{
  std::vector<dart::dynamics::SkeletonPtr> boxSkels(numBoxes);
  for (auto i = 0u; i < numBoxes; ++i)
    boxSkels[i] = createBox(
        Eigen::Vector3d(0.0, 0.0, heightFromGround + 0.25 + i * 0.5));
  return boxSkels;
}

//==============================================================================
dart::dynamics::SkeletonPtr createFloor()
{
  using namespace dart::dynamics;

  auto floor = Skeleton::create("floor");
  auto body = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  constexpr double floorWidth = 10.0;
  constexpr double floorHeight = 0.01;
  auto box = std::make_shared<BoxShape>(
      Eigen::Vector3d(floorWidth, floorWidth, floorHeight));
  auto* shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::LightGray());

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floorHeight / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

//==============================================================================
enum class LcpSolverType
{
  Dantzig,
  Pgs
};

//==============================================================================
struct BoxStackingState
{
  LcpSolverType solverType = LcpSolverType::Dantzig;
};

//==============================================================================
void applyLcpSolver(const dart::simulation::WorldPtr& world, LcpSolverType type)
{
  using namespace dart::constraint;

  if (type == LcpSolverType::Dantzig) {
    auto lcpSolver = std::make_shared<DantzigBoxedLcpSolver>();
    world->setConstraintSolver(
        std::make_unique<BoxedLcpConstraintSolver>(lcpSolver));
  } else {
    auto lcpSolver = std::make_shared<PgsBoxedLcpSolver>();
    world->setConstraintSolver(
        std::make_unique<BoxedLcpConstraintSolver>(lcpSolver));
  }
}

} // namespace

//==============================================================================
DemoScene makeBoxStackingScene()
{
  DemoScene scene;
  scene.id = "box_stacking";
  scene.title = "Box Stacking";
  scene.category = "Constraints & Joints";
  scene.summary = "Stacked boxes comparing Dantzig and PGS LCP solvers.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();
    world->addSkeleton(createFloor());

    auto boxSkels = createBoxStack(5);
    for (const auto& boxSkel : boxSkels)
      world->addSkeleton(boxSkel);

    auto state = std::make_shared<BoxStackingState>();
    applyLcpSolver(world, state->solverType);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(12.0, 12.0, 9.0),
        ::osg::Vec3d(0.0, 0.0, 2.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.renderPanel = [world, state] {
      ImGui::Text("Time: %.3f", world->getTime());
      ImGui::TextUnformatted("LCP solver:");

      int solverIndex = state->solverType == LcpSolverType::Dantzig ? 0 : 1;
      bool clicked = false;
      clicked |= ImGui::RadioButton("Dantzig", &solverIndex, 0);
      ImGui::SameLine();
      clicked |= ImGui::RadioButton("PGS", &solverIndex, 1);
      // RadioButton returns true on any click, including re-clicking the
      // already-selected option. Only swap the solver when the selection truly
      // changes: World::setConstraintSolver rebuilds the solver and wakes every
      // sleeping skeleton, so a redundant re-apply would perturb the sim.
      if (clicked) {
        const LcpSolverType newType
            = solverIndex == 0 ? LcpSolverType::Dantzig : LcpSolverType::Pgs;
        if (newType != state->solverType) {
          state->solverType = newType;
          applyLcpSolver(world, state->solverType);
        }
      }

      ImGui::TextWrapped(
          "5 boxes free-fall onto a floor plane; switch the boxed-LCP "
          "solver to compare stacking stability.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
