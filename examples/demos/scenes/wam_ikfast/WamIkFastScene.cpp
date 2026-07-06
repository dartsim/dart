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

// Ported from examples/wam_ikfast: a WAM arm posed purely kinematically by
// IkFast-accelerated whole-body IK (SharedLibraryIkFast dlopens a small
// shared library built from ikfast-generated code), with every body node
// draggable (Alt/Ctrl/Shift + click) and a togglable IK target gizmo on the
// end effector.
//
// Deviations from the original: the original builds `libwamIk.so` from
// ikfast/ikfast71.Transform6D.4_6_9_10_11_12_f8.cpp via its own `wamIk` CMake
// target; that source is duplicated here (ikfast/ subdirectory) under a
// different target/library name ("dartDemosWamIk") so this scene has no
// build dependency on examples/wam_ikfast/, which a later cleanup phase may
// retire (see ikfast/CMakeLists.txt). The original's InputHandler
// (p/P) printed every DOF name and position to stdout, which would be
// invisible in a windowed consolidated app; here it is routed to the host's
// diagnostics log instead. The rest configuration and each end effector's
// default IK bounds/target transform are captured once at scene-build time
// (right after setupEndEffectors(), exactly matching the original's
// InputHandler::initialize() timing relative to the rest of main()) rather
// than in a constructor that runs after the world node is attached, since
// nothing here depends on viewer/world-node attachment order. Per-body and
// per-target drag-and-drop are registered in onActivate and torn down on
// scene switch, matching every other B3 DnD scene. viewer->allowSimulation
// (false) is applied here (as the original does) to keep the host's global
// Play button from stepping physics under this purely-kinematic scene;
// restored to true on teardown so later (non-kinematic) scenes are
// unaffected.

#include "../Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/urdf/urdf.hpp>

#include <dart/dart.hpp>

#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dart_demos {

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::EndEffector;
using dart::dynamics::SkeletonPtr;

//==============================================================================
dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create("ground");
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  const double thickness = 0.01;
  tf.translation() = Eigen::Vector3d(0, 0, -thickness / 2.0);
  dart::dynamics::WeldJoint::Properties joint;
  joint.mT_ParentBodyToJoint = tf;
  ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr, joint);
  auto groundShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(10, 10, thickness));
  auto* shapeNode = ground->getBodyNode(0)
                        ->createShapeNodeWith<
                            dart::dynamics::VisualAspect,
                            dart::dynamics::CollisionAspect,
                            dart::dynamics::DynamicsAspect>(groundShape);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue(0.2));
  return ground;
}

//==============================================================================
std::string ikFastLibraryName()
{
  std::stringstream ss;
  // An absolute path, not a bare filename: SharedLibraryManager::load()
  // resolves the given string via std::ifstream relative to the process's
  // current working directory rather than dlopen()'s own RPATH search, so a
  // bare filename would only resolve when dart-demos happens to be launched
  // from DART_DEMOS_IKFAST_DIR itself (see CMakeLists.txt).
  ss << DART_DEMOS_IKFAST_DIR "/" << DART_SHARED_LIB_PREFIX << "dartDemosWamIk";
#if (DART_OS_LINUX || DART_OS_MACOS || DART_OS_FREEBSD) && DART_BUILD_MODE_DEBUG
  ss << "d";
#endif
  ss << "." << DART_SHARED_LIB_EXTENSION;
  return ss.str();
}

//==============================================================================
/// Per-instance state captured by this scene's onActivate/key-action lambdas.
struct WamIkFastState
{
  SkeletonPtr wam;
  dart::gui::osg::InteractiveFramePtr target;

  Eigen::VectorXd restConfig;

  // The single "ee" end effector's toggle state, custom (infinite) default
  // bounds, and default target transform, captured once at build time and
  // restored when the target gizmo is toggled off. The original tracked these
  // per end effector; this scene has exactly one, so they are plain scalars.
  bool constraintActive = false;
  std::pair<Eigen::Vector6d, Eigen::Vector6d> defaultBounds;
  Eigen::Isometry3d defaultTargetTf = Eigen::Isometry3d::Identity();

  // The IkFast gradient method, so onActivate can report a dlopen failure
  // (the analytical IK silently no-ops without its shared library).
  dart::dynamics::IkFast* ikFast = nullptr;
  std::function<void(const std::string&)> log;
};

//==============================================================================
void printDofValues(WamIkFastState& state)
{
  if (!state.log)
    return;
  for (std::size_t i = 0; i < state.wam->getNumDofs(); ++i) {
    std::ostringstream ss;
    ss << state.wam->getDof(i)->getName() << ": "
       << state.wam->getDof(i)->getPosition();
    state.log(ss.str());
  }
}

//==============================================================================
void toggleTarget(
    WamIkFastState& state, const dart::simulation::WorldPtr& world)
{
  auto* ee = state.wam->getEndEffector("ee");
  if (!ee)
    return;
  auto ik = ee->getIK();
  if (!ik)
    return;

  if (state.constraintActive) {
    state.constraintActive = false;
    ik->getErrorMethod().setBounds(
        state.defaultBounds.first, state.defaultBounds.second);
    ik->getTarget()->setRelativeTransform(state.defaultTargetTf);
    world->removeSimpleFrame(ik->getTarget());
  } else {
    state.constraintActive = true;
    // Reset to the InverseKinematics::ErrorMethod's own standard defaults
    // (not the scene's custom infinite bounds) while the target is active,
    // exactly as the original did.
    ik->getErrorMethod().setBounds();
    ik->getTarget()->setTransform(ee->getTransform());
    world->addSimpleFrame(ik->getTarget());
  }
}

} // namespace

//==============================================================================
DemoScene makeWamIkFastScene()
{
  DemoScene scene;
  scene.id = "wam_ikfast";
  scene.title = "WAM IkFast";
  scene.category = "Control & IK";
  scene.summary
      = "WAM arm posed by IkFast analytical whole-body IK, fully draggable.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();
    // The original app forbids all physics; here viewer->allowSimulation(false)
    // only blocks the Play path, while the Step button, --cycle-scenes, and
    // --headless settle steps call world->step() directly. Zero gravity makes
    // any such accidental step inert so the unactuated FORCE-joint arm holds
    // its kinematic IK pose instead of free-falling in catalog screenshots.
    world->setGravity(Eigen::Vector3d::Zero());

    dart::utils::DartLoader urdfParser;
    urdfParser.addPackageDirectory(
        "herb_description", DART_DATA_PATH "/urdf/wam");
    auto wam = urdfParser.parseSkeleton(DART_DATA_PATH "/urdf/wam/wam.urdf");
    if (!wam)
      throw std::runtime_error("failed to load " DART_DATA_PATH
                               "/urdf/wam/wam.urdf");

    for (const char* name : {"/j1", "/j2", "/j3", "/j4", "/j5", "/j6", "/j7"}) {
      if (auto* dof = wam->getDof(name))
        dof->setPosition(0.0);
    }

    const Eigen::Vector3d infiniteLinear
        = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    const Eigen::Vector3d infiniteAngular
        = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

    Eigen::Isometry3d tfHand(Eigen::Isometry3d::Identity());
    tfHand.translate(Eigen::Vector3d(0.0, 0.0, -0.09));

    auto* wam7 = wam->getBodyNode("/wam7");
    if (!wam7)
      throw std::runtime_error("wam.urdf: missing body node /wam7");
    auto* ee = wam7->createEndEffector("ee");
    ee->setDefaultRelativeTransform(tfHand, true);

    auto target = std::make_shared<dart::gui::osg::InteractiveFrame>(
        dart::dynamics::Frame::World(), "lh_target");
    ee->getIK(true)->setTarget(target);

    const std::vector<std::size_t> ikFastDofs{0, 1, 3, 4, 5, 6};
    const std::vector<std::size_t> ikFastFreeDofs{2};
    auto& ikFast
        = ee->getIK()->setGradientMethod<dart::dynamics::SharedLibraryIkFast>(
            ikFastLibraryName(), ikFastDofs, ikFastFreeDofs);
    ee->getIK()->getErrorMethod().setLinearBounds(
        -infiniteLinear, infiniteLinear);
    ee->getIK()->getErrorMethod().setAngularBounds(
        -infiniteAngular, infiniteAngular);

    world->addSkeleton(wam);
    world->addSkeleton(createGround());

    auto state = std::make_shared<WamIkFastState>();
    state->wam = wam;
    state->target = target;
    state->ikFast = &ikFast;
    state->restConfig = wam->getPositions();
    // Captured after setGradientMethod/setLinearBounds/setAngularBounds above,
    // exactly like the original's InputHandler::initialize() -- these are the
    // scene's own custom (infinite) bounds and the target's initial (identity)
    // relative transform, restored when the target is toggled off.
    state->defaultBounds = ee->getIK()->getErrorMethod().getBounds();
    state->defaultTargetTf = ee->getIK()->getTarget()->getRelativeTransform();

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(5.34, 3.00, 1.91),
        ::osg::Vec3d(0.00, 0.00, 0.50),
        ::osg::Vec3d(0.00, 0.00, 0.98)};

    setup.onActivate = [state](DemoHostContext& ctx) {
      auto* viewer = ctx.viewer();
      state->log = [ctx](const std::string& message) {
        ctx.log(message);
      };

      // Purely kinematic: keep the host's global Play button from stepping
      // physics under this scene (restored on teardown).
      viewer->allowSimulation(false);
      ctx.addTeardown([viewer] { viewer->allowSimulation(true); });

      // Force the lazy IkFast dlopen/configure (solveAndApply triggers it) and
      // surface a load failure in the Diagnostics log: without the shared
      // library the analytical IK silently degrades to a no-op, which would
      // otherwise be invisible.
      state->wam->getIK(true)->solveAndApply(true);
      if (state->ikFast && !state->ikFast->isConfigured())
        ctx.log(
            "wam_ikfast: failed to load the IkFast shared library ("
            + ikFastLibraryName()
            + "); analytical IK is unavailable and the arm will not track "
              "the target.");

      for (std::size_t i = 0; i < state->wam->getNumBodyNodes(); ++i) {
        auto* dnd = viewer->enableDragAndDrop(
            state->wam->getBodyNode(i), false, false);
        if (dnd)
          ctx.addTeardown([viewer, dnd] { viewer->disableDragAndDrop(dnd); });
      }

      if (auto* dnd = viewer->enableDragAndDrop(state->target.get()))
        ctx.addTeardown([viewer, dnd] { viewer->disableDragAndDrop(dnd); });
    };

    setup.renderPanel = [state] {
      state->wam->getIK(true)->solveAndApply(true);

      ImGui::Text(
          "IK target: %s", state->constraintActive ? "active" : "inactive");
      ImGui::TextWrapped(
          "Purely kinematic (physics stepping is disabled for this scene). "
          "Alt+drag translates a body, Ctrl+drag rotates it, Shift+drag "
          "moves it via its parent joint. '1' toggles the draggable IK "
          "target on the end effector; 't' resets to the rest pose; 'p' "
          "prints DOF values to the Diagnostics log.");
    };

    setup.keyActions.push_back(KeyAction{'p', "Print DOF values", [state] {
                                           printDofValues(*state);
                                         }});
    setup.keyActions.push_back(KeyAction{'t', "Reset to rest pose", [state] {
                                           state->wam->setPositions(
                                               state->restConfig);
                                         }});
    setup.keyActions.push_back(
        KeyAction{'1', "Toggle IK target", [state, world] {
                    toggleTarget(*state, world);
                  }});

    return setup;
  };

  return scene;
}

} // namespace dart_demos
