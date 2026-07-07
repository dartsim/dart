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

// Ported from examples/simulation_event_handler: a small fixed set of rigid
// bodies (ground, 3 boxes, 1 sphere); Tab/Backspace select a body, arrow keys
// and u/d apply forces, q/w/e/a/z/c apply torques, +/- scale the magnitude,
// and 'v' toggles force/torque arrow visualization.
//
// Deviations from the original: 's'/'r' (step/reset) and '<'/'>' (timestep)
// are dropped -- dart-demos' host-level Simulation toolbar exposes Step, Reset,
// and a Timestep slider generically for every scene, so a per-scene key for the
// same thing would just contend with it. 'i'/'h' (print
// state/help to stdout) are dropped too: the scene panel already shows the
// same information continuously, and there is no user-facing console in the
// GUI. Force/torque arrows are drawn with a SimpleFrame + ArrowShape pool
// added to the world (as RigidCubesScene's contact-force visuals do), rather
// than the original's raw OSG geometry attached directly to the viewer's root
// group, so they are automatically cleaned up when the scene's world is torn
// down.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <memory>
#include <vector>

#include <cmath>

namespace dart_demos {

namespace {

constexpr std::size_t kArrowUpdateFrequency = 5;
constexpr double kDefaultForceMagnitude = 100.0;
constexpr double kDefaultTorqueMagnitude = 50.0;

//==============================================================================
dart::dynamics::SkeletonPtr createBox(
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& color,
    double mass = 1.0)
{
  using namespace dart::dynamics;

  auto skeleton = Skeleton::create(name);
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  pair.first->setTransformFromParentBodyNode(tf);
  pair.second->setName(name + "_body");

  auto boxShape = std::make_shared<BoxShape>(size);
  auto shapeNode = pair.second->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(color);

  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(boxShape->computeInertia(mass));
  pair.second->setInertia(inertia);

  return skeleton;
}

//==============================================================================
dart::dynamics::SkeletonPtr createSphere(
    const std::string& name,
    const Eigen::Vector3d& position,
    double radius,
    const Eigen::Vector3d& color,
    double mass = 1.0)
{
  using namespace dart::dynamics;

  auto skeleton = Skeleton::create(name);
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  pair.first->setTransformFromParentBodyNode(tf);
  pair.second->setName(name + "_body");

  auto sphereShape = std::make_shared<SphereShape>(radius);
  auto shapeNode = pair.second->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(sphereShape);
  shapeNode->getVisualAspect()->setColor(color);

  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(sphereShape->computeInertia(mass));
  pair.second->setInertia(inertia);

  return skeleton;
}

//==============================================================================
dart::dynamics::SkeletonPtr createGround()
{
  using namespace dart::dynamics;

  auto ground = Skeleton::create("ground");
  auto pair = ground->createJointAndBodyNodePair<WeldJoint>(nullptr);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.05);
  pair.first->setTransformFromParentBodyNode(tf);
  pair.second->setName("ground_body");

  constexpr double groundSize = 10.0;
  constexpr double groundHeight = 0.1;
  auto groundShape = std::make_shared<BoxShape>(
      Eigen::Vector3d(groundSize, groundSize, groundHeight));
  auto shapeNode = pair.second->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(groundShape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.3, 0.3, 0.3));

  return ground;
}

//==============================================================================
/// A rigid body (any joint with dofs) that can be selected and pushed/torqued.
std::vector<dart::dynamics::BodyNode*> getRigidBodies(
    const dart::simulation::WorldPtr& world)
{
  std::vector<dart::dynamics::BodyNode*> bodies;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto& skel = world->getSkeleton(i);
    for (std::size_t j = 0; j < skel->getNumBodyNodes(); ++j) {
      auto* body = skel->getBodyNode(j);
      auto* joint = body->getParentJoint();
      if (joint && joint->getNumDofs() > 0)
        bodies.push_back(body);
    }
  }
  return bodies;
}

//==============================================================================
struct AppliedVector
{
  dart::dynamics::BodyNode* body;
  Eigen::Vector3d vector;
};

//==============================================================================
struct SimulationEventHandlerState
{
  std::vector<dart::dynamics::BodyNode*> rigidBodies;
  std::size_t selectedIndex = 0;

  double forceMagnitude = kDefaultForceMagnitude;
  double torqueMagnitude = kDefaultTorqueMagnitude;
  bool showForceArrows = true;

  std::vector<AppliedVector> pendingForces;
  std::vector<AppliedVector> pendingTorques;
  std::size_t frameCounter = 0;

  std::vector<dart::dynamics::SimpleFramePtr> arrowFrames;
  std::vector<std::shared_ptr<dart::dynamics::ArrowShape>> arrowShapes;
};

//==============================================================================
dart::dynamics::BodyNode* selectedBody(SimulationEventHandlerState& state)
{
  if (state.rigidBodies.empty())
    return nullptr;
  return state.rigidBodies[state.selectedIndex];
}

//==============================================================================
void applyForce(
    SimulationEventHandlerState& state, const Eigen::Vector3d& force)
{
  auto* body = selectedBody(state);
  if (!body)
    return;
  body->addExtForce(force);
  state.pendingForces.push_back({body, force});
}

//==============================================================================
void applyTorque(
    SimulationEventHandlerState& state, const Eigen::Vector3d& torque)
{
  auto* body = selectedBody(state);
  if (!body)
    return;
  body->addExtTorque(torque);
  state.pendingTorques.push_back({body, torque});
}

//==============================================================================
void selectNextBody(SimulationEventHandlerState& state)
{
  if (state.rigidBodies.empty())
    return;
  state.selectedIndex = (state.selectedIndex + 1) % state.rigidBodies.size();
}

//==============================================================================
void selectPreviousBody(SimulationEventHandlerState& state)
{
  if (state.rigidBodies.empty())
    return;
  state.selectedIndex = state.selectedIndex == 0 ? state.rigidBodies.size() - 1
                                                 : state.selectedIndex - 1;
}

//==============================================================================
void ensureArrowVisuals(
    SimulationEventHandlerState& state,
    const dart::simulation::WorldPtr& world,
    std::size_t count)
{
  while (state.arrowFrames.size() < count) {
    auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World());
    auto arrow = std::make_shared<dart::dynamics::ArrowShape>(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::UnitZ() * 0.01,
        dart::dynamics::ArrowShape::Properties(0.006, 2.0, 0.15),
        Eigen::Vector4d(0.9, 0.1, 0.1, 1.0));
    frame->setShape(arrow);
    frame->getVisualAspect(true)->setHidden(true);
    world->addSimpleFrame(frame);
    state.arrowFrames.push_back(frame);
    state.arrowShapes.push_back(arrow);
  }
}

//==============================================================================
void hideArrowsFrom(SimulationEventHandlerState& state, std::size_t start)
{
  for (std::size_t i = start; i < state.arrowFrames.size(); ++i)
    state.arrowFrames[i]->getVisualAspect(true)->setHidden(true);
}

//==============================================================================
/// Hides every arrow visual immediately and drops any queued force/torque
/// draws. Used by the toggle-off paths so the arrows disappear the instant the
/// user turns them off, even while the simulation is paused (postStep, which
/// normally refreshes them, does not run while paused).
void hideAllArrows(SimulationEventHandlerState& state)
{
  hideArrowsFrom(state, 0);
  state.pendingForces.clear();
  state.pendingTorques.clear();
}

//==============================================================================
// Redraw arrows for every pending force/torque (each scaled to a fixed 0.5 m
// visual length from the body's COM, as in the original), then clear the
// pending lists -- this runs every kArrowUpdateFrequency steps, matching the
// original's frame-throttled refresh.
void updateArrowVisuals(
    SimulationEventHandlerState& state, const dart::simulation::WorldPtr& world)
{
  if (!state.showForceArrows) {
    hideArrowsFrom(state, 0);
    state.pendingForces.clear();
    state.pendingTorques.clear();
    return;
  }

  const std::size_t count
      = state.pendingForces.size() + state.pendingTorques.size();
  ensureArrowVisuals(state, world, count);

  std::size_t idx = 0;
  auto draw = [&](const AppliedVector& v) {
    if (v.vector.norm() < 1e-6)
      return;
    const Eigen::Vector3d com = v.body->getCOM();
    const Eigen::Vector3d tip = com + v.vector.normalized() * 0.5;
    auto* visual = state.arrowFrames[idx]->getVisualAspect(true);
    visual->setHidden(false);
    state.arrowShapes[idx]->setPositions(com, tip);
    ++idx;
  };
  for (const auto& f : state.pendingForces)
    draw(f);
  for (const auto& t : state.pendingTorques)
    draw(t);

  hideArrowsFrom(state, idx);
  state.pendingForces.clear();
  state.pendingTorques.clear();
}

} // namespace

//==============================================================================
DemoScene makeSimulationEventHandlerScene()
{
  DemoScene scene;
  scene.id = "simulation_event_handler";
  scene.title = "Simulation Events";
  scene.category = "Rigid Body";
  scene.summary
      = "Force and torque arrows applied to a keyboard-selected rigid body.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world->setTimeStep(0.001);

    world->addSkeleton(createGround());
    world->addSkeleton(createBox(
        "box1",
        Eigen::Vector3d(-1.0, 0.0, 2.0),
        Eigen::Vector3d(0.4, 0.4, 0.4),
        Eigen::Vector3d(0.8, 0.2, 0.2)));
    world->addSkeleton(createBox(
        "box2",
        Eigen::Vector3d(0.0, 0.0, 3.0),
        Eigen::Vector3d(0.6, 0.3, 0.3),
        Eigen::Vector3d(0.2, 0.8, 0.2)));
    world->addSkeleton(createSphere(
        "sphere1",
        Eigen::Vector3d(1.0, 0.0, 2.5),
        0.3,
        Eigen::Vector3d(0.2, 0.2, 0.8)));
    world->addSkeleton(createBox(
        "box3",
        Eigen::Vector3d(0.5, 1.0, 2.2),
        Eigen::Vector3d(0.3, 0.5, 0.4),
        Eigen::Vector3d(0.9, 0.9, 0.2)));

    auto state = std::make_shared<SimulationEventHandlerState>();
    state->rigidBodies = getRigidBodies(world);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(5.0, 5.0, 3.0),
        ::osg::Vec3d(0.0, 0.0, 1.0),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.postStep = [world, state] {
      ++state->frameCounter;
      if (state->frameCounter % kArrowUpdateFrequency == 0)
        updateArrowVisuals(*state, world);
    };

    setup.keyActions.push_back(KeyAction{
        static_cast<int>(::osgGA::GUIEventAdapter::KEY_Tab),
        "Select next body",
        [state] {
          selectNextBody(*state);
        }});
    setup.keyActions.push_back(KeyAction{
        static_cast<int>(::osgGA::GUIEventAdapter::KEY_BackSpace),
        "Select previous body",
        [state] {
          selectPreviousBody(*state);
        }});
    setup.keyActions.push_back(KeyAction{
        static_cast<int>(::osgGA::GUIEventAdapter::KEY_Up),
        "Force +Y",
        [state] {
          applyForce(*state, Eigen::Vector3d(0, state->forceMagnitude, 0));
        }});
    setup.keyActions.push_back(KeyAction{
        static_cast<int>(::osgGA::GUIEventAdapter::KEY_Down),
        "Force -Y",
        [state] {
          applyForce(*state, Eigen::Vector3d(0, -state->forceMagnitude, 0));
        }});
    setup.keyActions.push_back(KeyAction{
        static_cast<int>(::osgGA::GUIEventAdapter::KEY_Left),
        "Force -X",
        [state] {
          applyForce(*state, Eigen::Vector3d(-state->forceMagnitude, 0, 0));
        }});
    setup.keyActions.push_back(KeyAction{
        static_cast<int>(::osgGA::GUIEventAdapter::KEY_Right),
        "Force +X",
        [state] {
          applyForce(*state, Eigen::Vector3d(state->forceMagnitude, 0, 0));
        }});
    setup.keyActions.push_back(KeyAction{
        'u', "Force +Z (up)", [state] {
          applyForce(*state, Eigen::Vector3d(0, 0, state->forceMagnitude));
        }});
    setup.keyActions.push_back(KeyAction{
        'd', "Force -Z (down)", [state] {
          applyForce(*state, Eigen::Vector3d(0, 0, -state->forceMagnitude));
        }});
    setup.keyActions.push_back(KeyAction{
        'q', "Torque +X", [state] {
          applyTorque(*state, Eigen::Vector3d(state->torqueMagnitude, 0, 0));
        }});
    setup.keyActions.push_back(KeyAction{
        'w', "Torque +Y", [state] {
          applyTorque(*state, Eigen::Vector3d(0, state->torqueMagnitude, 0));
        }});
    setup.keyActions.push_back(KeyAction{
        'e', "Torque +Z", [state] {
          applyTorque(*state, Eigen::Vector3d(0, 0, state->torqueMagnitude));
        }});
    setup.keyActions.push_back(KeyAction{
        'a', "Torque -X", [state] {
          applyTorque(*state, Eigen::Vector3d(-state->torqueMagnitude, 0, 0));
        }});
    setup.keyActions.push_back(KeyAction{
        'z', "Torque -Y", [state] {
          applyTorque(*state, Eigen::Vector3d(0, -state->torqueMagnitude, 0));
        }});
    setup.keyActions.push_back(KeyAction{
        'c', "Torque -Z", [state] {
          applyTorque(*state, Eigen::Vector3d(0, 0, -state->torqueMagnitude));
        }});
    setup.keyActions.push_back(
        KeyAction{'=', "Increase magnitude", [state] {
                    state->forceMagnitude
                        = std::min(state->forceMagnitude * 1.5, 10000.0);
                    state->torqueMagnitude
                        = std::min(state->torqueMagnitude * 1.5, 5000.0);
                  }});
    setup.keyActions.push_back(KeyAction{
        '-', "Decrease magnitude", [state] {
          state->forceMagnitude = std::max(state->forceMagnitude * 0.67, 0.1);
          state->torqueMagnitude = std::max(state->torqueMagnitude * 0.67, 0.1);
        }});
    setup.keyActions.push_back(KeyAction{'v', "Toggle force arrows", [state] {
                                           state->showForceArrows
                                               = !state->showForceArrows;
                                           if (!state->showForceArrows)
                                             hideAllArrows(*state);
                                         }});

    setup.renderPanel = [world, state] {
      ImGui::Text("Rigid bodies: %zu", state->rigidBodies.size());
      if (auto* body = selectedBody(*state))
        ImGui::Text(
            "Selected: %s (%zu/%zu)",
            body->getName().c_str(),
            state->selectedIndex + 1,
            state->rigidBodies.size());
      else
        ImGui::TextUnformatted("Selected: (none)");

      auto force = static_cast<float>(state->forceMagnitude);
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
      if (ImGui::SliderFloat(
              "Force mag",
              &force,
              1.0f,
              500.0f,
              "%.1f",
              ImGuiSliderFlags_AlwaysClamp)
          && std::isfinite(force))
        state->forceMagnitude = std::clamp(force, 1.0f, 500.0f);

      auto torque = static_cast<float>(state->torqueMagnitude);
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
      if (ImGui::SliderFloat(
              "Torque mag",
              &torque,
              1.0f,
              200.0f,
              "%.1f",
              ImGuiSliderFlags_AlwaysClamp)
          && std::isfinite(torque))
        state->torqueMagnitude = std::clamp(torque, 1.0f, 200.0f);

      bool showArrows = state->showForceArrows;
      if (ImGui::Checkbox("Show force/torque arrows", &showArrows)) {
        state->showForceArrows = showArrows;
        if (!showArrows)
          hideAllArrows(*state);
      }

      ImGui::TextWrapped(
          "Tab/Backspace select a body; arrow keys and u/d push it; "
          "q/w/e/a/z/c apply torque; +/- scale the magnitude.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
