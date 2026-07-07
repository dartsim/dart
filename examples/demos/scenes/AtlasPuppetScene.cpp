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

// Ported from examples/atlas_puppet: a purely kinematic Atlas whole-body IK
// puppet. Four end effectors (l_hand, r_hand, l_foot, r_foot) can each be
// driven by a draggable InteractiveFrame target; a RelaxedPosture objective
// and BalanceConstraint keep the robot upright while W/A/S/D/Q/E/F/Z walk it
// around the scene and 'R' asks the solver to optimize posture/balance.
//
// Deviations from the original: the original drives everything from
// WorldNode::customPreRefresh() (render-rate, since
// viewer.allowSimulation(false) disables customPreStep entirely). This host
// has no per-render-frame scene hook, so the same per-frame movement +
// solveAndApply logic runs at the top of renderPanel instead -- the same
// trick already used by wam_ikfast (B3) for an identical
// kinematic-IK-at-render-rate scene, and it is called at the same cadence
// (once per rendered frame) as the original. viewer->allowSimulation(false)
// is applied in onActivate and restored to true on teardown, matching
// wam_ikfast. Gravity is zeroed (the original never steps physics at all, so
// this has no equivalent, but it keeps --cycle-scenes/--headless/the Step
// button -- which call world->step() directly, bypassing allowSimulation --
// from making the unactuated robot collapse in a forced step). The
// w/a/s/d/q/e/f/z movement keys and the 'r' posture-optimize hold need both
// KEYDOWN and KEYUP, which DemoScene::KeyAction (key-down only) cannot
// express; they are handled by a small osgGA::GUIEventHandler registered via
// DemoHostContext::addEventHandler, the same pattern
// OperationalSpaceControlScene (B3) used for its held 1/2/3 constraint keys.
// The single-press keys (1-4 target toggle, t reset, x/c support toggle, p
// print) are ordinary KeyActions, auto-mirrored as panel buttons. 'p''s
// stdout dump is routed to the Diagnostics log instead (a windowed app has no
// visible console), matching wam_ikfast's 'p'. getEqConstraint(1)'s fragile
// hard-coded index is kept as in the original (see the parity portRisk); it
// is set up by this same factory a few lines above, so the index is stable
// here.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/urdf/urdf.hpp>

#include <dart/dart.hpp>

#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace dart_demos {

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::EndEffector;
using dart::dynamics::Frame;
using dart::dynamics::SkeletonPtr;

constexpr double kDisplayElevation = 0.05;

//==============================================================================
/// Ported verbatim from the original's RelaxedPosture: a soft objective that
/// nudges the posture back inside [lower, upper] (or all the way to `ideal`
/// while `enforceIdealPosture` is held true, i.e. while 'r' is held).
class RelaxedPosture : public dart::optimizer::Function
{
public:
  RelaxedPosture(
      const Eigen::VectorXd& idealPosture,
      const Eigen::VectorXd& lower,
      const Eigen::VectorXd& upper,
      const Eigen::VectorXd& weights)
    : enforceIdealPosture(false),
      mIdeal(idealPosture),
      mLower(lower),
      mUpper(upper),
      mWeights(weights)
  {
    mResultVector.setZero(mIdeal.size());
  }

  double eval(const Eigen::VectorXd& x) override
  {
    computeResultVector(x);
    return 0.5 * mResultVector.dot(mResultVector);
  }

  void evalGradient(
      const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) override
  {
    computeResultVector(x);
    grad.setZero();
    const int smaller = std::min<int>(mResultVector.size(), grad.size());
    for (int i = 0; i < smaller; ++i)
      grad[i] = mResultVector[i];
  }

  void computeResultVector(const Eigen::VectorXd& x)
  {
    mResultVector.setZero();
    if (enforceIdealPosture) {
      for (int i = 0; i < x.size() && i < mIdeal.size(); ++i)
        mResultVector[i] = mWeights[i] * (x[i] - mIdeal[i]);
    } else {
      for (int i = 0; i < x.size() && i < mIdeal.size(); ++i) {
        if (x[i] < mLower[i])
          mResultVector[i] = mWeights[i] * (x[i] - mLower[i]);
        else if (mUpper[i] < x[i])
          mResultVector[i] = mWeights[i] * (x[i] - mUpper[i]);
      }
    }
  }

  bool enforceIdealPosture;

protected:
  Eigen::VectorXd mResultVector;
  Eigen::VectorXd mIdeal;
  Eigen::VectorXd mLower;
  Eigen::VectorXd mUpper;
  Eigen::VectorXd mWeights;
};

//==============================================================================
enum MoveComponent
{
  MoveQ = 0,
  MoveW,
  MoveE,
  MoveA,
  MoveS,
  MoveD,
  MoveF,
  MoveZ,
  NumMoveComponents
};

//==============================================================================
/// Per-instance state captured by this scene's renderPanel/key-action lambdas
/// and the move handler.
struct AtlasPuppetState
{
  SkeletonPtr atlas;

  Eigen::VectorXd restConfig;

  std::vector<bool> moveComponents
      = std::vector<bool>(NumMoveComponents, false);

  // Per-EE state, gathered in end-effector iteration order (l_hand, r_hand,
  // l_foot, r_foot), matching the original's InputHandler::initialize().
  std::vector<std::size_t> endEffectorIndex;
  std::vector<std::pair<Eigen::Vector6d, Eigen::Vector6d>> defaultBounds;
  dart::common::aligned_vector<Eigen::Isometry3d> defaultTargetTf;
  std::vector<bool> constraintActive;

  std::shared_ptr<RelaxedPosture> posture;
  std::shared_ptr<dart::constraint::BalanceConstraint> balance;

  std::size_t consecutiveFailures = 0;
  bool reportedFailing = false;

  std::function<void(const std::string&)> log;
};

//==============================================================================
SkeletonPtr createGround()
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
SkeletonPtr createAtlas()
{
  dart::utils::DartLoader urdf;
  auto atlas
      = urdf.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf");
  if (!atlas)
    return nullptr;

  const double scale = 0.25;
  auto boxShape = std::make_shared<dart::dynamics::BoxShape>(
      scale * Eigen::Vector3d(1.0, 1.0, 0.5));
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.1);
  auto* shapeNode
      = atlas->getBodyNode(0)
            ->createShapeNodeWith<dart::dynamics::VisualAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());
  shapeNode->setRelativeTransform(tf);

  return atlas;
}

//==============================================================================
using dart::math::constantsd;

void setupStartConfiguration(const SkeletonPtr& atlas)
{
  const double deg = constantsd::pi() / 180.0;

  atlas->getDof("r_leg_hpy")->setPosition(-45.0 * deg);
  atlas->getDof("r_leg_kny")->setPosition(90.0 * deg);
  atlas->getDof("r_leg_aky")->setPosition(-45.0 * deg);

  atlas->getDof("l_leg_hpy")->setPosition(-45.0 * deg);
  atlas->getDof("l_leg_kny")->setPosition(90.0 * deg);
  atlas->getDof("l_leg_aky")->setPosition(-45.0 * deg);

  atlas->getDof("r_arm_shx")->setPosition(65.0 * deg);
  atlas->getDof("r_arm_ely")->setPosition(90.0 * deg);
  atlas->getDof("r_arm_elx")->setPosition(-90.0 * deg);
  atlas->getDof("r_arm_wry")->setPosition(65.0 * deg);

  atlas->getDof("l_arm_shx")->setPosition(-65.0 * deg);
  atlas->getDof("l_arm_ely")->setPosition(90.0 * deg);
  atlas->getDof("l_arm_elx")->setPosition(90.0 * deg);
  atlas->getDof("l_arm_wry")->setPosition(65.0 * deg);

  atlas->getDof("r_leg_kny")->setPositionLowerLimit(10.0 * deg);
  atlas->getDof("l_leg_kny")->setPositionLowerLimit(10.0 * deg);
}

//==============================================================================
void setupEndEffectors(const SkeletonPtr& atlas)
{
  Eigen::VectorXd rootJointWeights = 0.01 * Eigen::VectorXd::Ones(6);

  Eigen::Vector3d linearBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d angularBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  Eigen::Isometry3d tfHand(Eigen::Isometry3d::Identity());
  tfHand.translation() = Eigen::Vector3d(0.0009, 0.1254, 0.012);
  tfHand.rotate(Eigen::AngleAxisd(
      90.0 * constantsd::pi() / 180.0, Eigen::Vector3d::UnitZ()));

  auto* lHand = atlas->getBodyNode("l_hand")->createEndEffector("l_hand");
  lHand->setDefaultRelativeTransform(tfHand, true);
  auto lhTarget = std::make_shared<dart::gui::osg::InteractiveFrame>(
      Frame::World(), "lh_target");
  lHand->getIK(true)->setTarget(lhTarget);
  lHand->getIK()->useWholeBody();
  lHand->getIK()->getGradientMethod().setComponentWeights(rootJointWeights);
  lHand->getIK()->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  lHand->getIK()->getErrorMethod().setAngularBounds(
      -angularBounds, angularBounds);

  tfHand.translation()[0] = -tfHand.translation()[0];
  tfHand.translation()[1] = -tfHand.translation()[1];
  tfHand.linear() = tfHand.linear().inverse().eval();

  auto* rHand = atlas->getBodyNode("r_hand")->createEndEffector("r_hand");
  rHand->setDefaultRelativeTransform(tfHand, true);
  auto rhTarget = std::make_shared<dart::gui::osg::InteractiveFrame>(
      Frame::World(), "rh_target");
  rHand->getIK(true)->setTarget(rhTarget);
  rHand->getIK()->useWholeBody();
  rHand->getIK()->getGradientMethod().setComponentWeights(rootJointWeights);
  rHand->getIK()->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  rHand->getIK()->getErrorMethod().setAngularBounds(
      -angularBounds, angularBounds);

  dart::math::SupportGeometry support;
  const double supPosX = 0.10 - 0.186;
  const double supNegX = -0.03 - 0.186;
  const double supPosY = 0.03;
  const double supNegY = -0.03;
  support.push_back(Eigen::Vector3d(supNegX, supNegY, 0.0));
  support.push_back(Eigen::Vector3d(supPosX, supNegY, 0.0));
  support.push_back(Eigen::Vector3d(supPosX, supPosY, 0.0));
  support.push_back(Eigen::Vector3d(supNegX, supPosY, 0.0));

  Eigen::Isometry3d tfFoot(Eigen::Isometry3d::Identity());
  tfFoot.translation() = Eigen::Vector3d(0.186, 0.0, -0.08);

  linearBounds[2] = 1e-8;
  angularBounds[0] = 1e-8;
  angularBounds[1] = 1e-8;

  auto* lFoot = atlas->getBodyNode("l_foot")->createEndEffector("l_foot");
  lFoot->setRelativeTransform(tfFoot);
  auto lfTarget = std::make_shared<dart::gui::osg::InteractiveFrame>(
      Frame::World(), "lf_target");
  lFoot->getIK(true)->setTarget(lfTarget);
  lFoot->getIK()->setHierarchyLevel(1);
  lFoot->getIK()->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  lFoot->getIK()->getErrorMethod().setAngularBounds(
      -angularBounds, angularBounds);
  lFoot->getSupport(true)->setGeometry(support);
  lFoot->getSupport()->setActive();

  auto* rFoot = atlas->getBodyNode("r_foot")->createEndEffector("r_foot");
  rFoot->setRelativeTransform(tfFoot);
  auto rfTarget = std::make_shared<dart::gui::osg::InteractiveFrame>(
      Frame::World(), "rf_target");
  rFoot->getIK(true)->setTarget(rfTarget);
  rFoot->getIK()->setHierarchyLevel(1);
  rFoot->getIK()->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  rFoot->getIK()->getErrorMethod().setAngularBounds(
      -angularBounds, angularBounds);
  rFoot->getSupport(true)->setGeometry(support);
  rFoot->getSupport()->setActive();

  const double heightChange = -rFoot->getWorldTransform().translation()[2];
  atlas->getDof(5)->setPosition(heightChange);

  lFoot->getIK()->getTarget()->setTransform(lFoot->getTransform());
  rFoot->getIK()->getTarget()->setTransform(rFoot->getTransform());
}

//==============================================================================
void setupWholeBodySolver(const SkeletonPtr& atlas)
{
  auto solver
      = std::dynamic_pointer_cast<dart::optimizer::GradientDescentSolver>(
          atlas->getIK(true)->getSolver());
  solver->setNumMaxIterations(10);

  const std::size_t nDofs = atlas->getNumDofs();
  const double defaultWeight = 0.01;
  Eigen::VectorXd weights = defaultWeight * Eigen::VectorXd::Ones(nDofs);
  weights[2] = 0.0;
  weights[3] = 0.0;
  weights[4] = 0.0;
  weights[6] *= 0.2;
  weights[7] *= 0.2;
  weights[8] *= 0.2;

  Eigen::VectorXd lowerPosture = Eigen::VectorXd::Constant(
      nDofs, -std::numeric_limits<double>::infinity());
  lowerPosture[0] = -0.35;
  lowerPosture[1] = -0.35;
  lowerPosture[5] = 0.600;
  lowerPosture[6] = -0.1;
  lowerPosture[7] = -0.1;
  lowerPosture[8] = -0.1;

  Eigen::VectorXd upperPosture = Eigen::VectorXd::Constant(
      nDofs, std::numeric_limits<double>::infinity());
  upperPosture[0] = 0.35;
  upperPosture[1] = 0.35;
  upperPosture[5] = 0.885;
  upperPosture[6] = 0.1;
  upperPosture[7] = 0.1;
  upperPosture[8] = 0.1;

  auto objective = std::make_shared<RelaxedPosture>(
      atlas->getPositions(), lowerPosture, upperPosture, weights);
  atlas->getIK()->setObjective(objective);

  auto balance
      = std::make_shared<dart::constraint::BalanceConstraint>(atlas->getIK());
  atlas->getIK()->getProblem()->addEqConstraint(balance);
  balance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);
  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_SUPPORT);
}

//==============================================================================
/// Composes the WASDQEFZ root-motion delta and re-solves whole-body IK; run
/// once per rendered frame (see the file comment for why this lives in
/// renderPanel rather than a per-step hook).
void updatePuppet(AtlasPuppetState& state)
{
  bool anyMovement = false;
  for (bool move : state.moveComponents)
    anyMovement = anyMovement || move;

  if (anyMovement) {
    const Eigen::Isometry3d oldTf
        = state.atlas->getBodyNode(0)->getWorldTransform();
    Eigen::Isometry3d newTf = Eigen::Isometry3d::Identity();

    Eigen::Vector3d forward = oldTf.linear().col(0);
    forward[2] = 0.0;
    forward = forward.norm() > 1e-10 ? forward.normalized()
                                     : Eigen::Vector3d::Zero();

    Eigen::Vector3d left = oldTf.linear().col(1);
    left[2] = 0.0;
    left = left.norm() > 1e-10 ? left.normalized() : Eigen::Vector3d::Zero();

    const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();

    constexpr double linearStep = 0.01;
    constexpr double elevationStep = 0.2 * linearStep;
    const double rotationalStep = 2.0 * constantsd::pi() / 180.0;

    if (state.moveComponents[MoveW])
      newTf.translate(linearStep * forward);
    if (state.moveComponents[MoveS])
      newTf.translate(-linearStep * forward);
    if (state.moveComponents[MoveA])
      newTf.translate(linearStep * left);
    if (state.moveComponents[MoveD])
      newTf.translate(-linearStep * left);
    if (state.moveComponents[MoveF])
      newTf.translate(elevationStep * up);
    if (state.moveComponents[MoveZ])
      newTf.translate(-elevationStep * up);
    if (state.moveComponents[MoveQ])
      newTf.rotate(Eigen::AngleAxisd(rotationalStep, up));
    if (state.moveComponents[MoveE])
      newTf.rotate(Eigen::AngleAxisd(-rotationalStep, up));

    newTf.pretranslate(oldTf.translation());
    newTf.rotate(oldTf.rotation());

    state.atlas->getJoint(0)->setPositions(
        dart::dynamics::FreeJoint::convertToPositions(newTf));
  }

  const bool solved = state.atlas->getIK(true)->solveAndApply(true);
  if (!solved) {
    ++state.consecutiveFailures;
  } else {
    state.consecutiveFailures = 0;
    state.reportedFailing = false;
  }
  if (state.consecutiveFailures >= 1000 && !state.reportedFailing) {
    state.reportedFailing = true;
    if (state.log)
      state.log("atlas_puppet: IK solver has failed 1000 consecutive times.");
  }
}

//==============================================================================
/// Ported from the original's InputHandler: WASDQEFZ set/clear per-axis move
/// flags on KEYDOWN/KEYUP; 'r' toggles posture-enforcement + balance mode
/// while held. Needs KEYUP as well as KEYDOWN, so it cannot be expressed as
/// an ordinary (key-down-only) KeyAction -- see the file comment.
class AtlasPuppetMoveHandler : public ::osgGA::GUIEventHandler
{
public:
  explicit AtlasPuppetMoveHandler(std::shared_ptr<AtlasPuppetState> state)
    : mState(std::move(state))
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getHandled())
      return false;

    const bool down = ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN;
    const bool up = ea.getEventType() == ::osgGA::GUIEventAdapter::KEYUP;
    if (!down && !up)
      return false;

    // Don't let typing in an ImGui field (e.g. the host's "Search demos..."
    // box) walk or rotate the puppet -- osgGA delivers key events to every
    // handler regardless of ImGui focus.
    if (ImGui::GetIO().WantCaptureKeyboard)
      return false;

    if (ea.getKey() == 'r') {
      if (mState->posture)
        mState->posture->enforceIdealPosture = down;
      if (mState->balance) {
        mState->balance->setErrorMethod(
            down ? dart::constraint::BalanceConstraint::OPTIMIZE_BALANCE
                 : dart::constraint::BalanceConstraint::FROM_CENTROID);
      }
      return true;
    }

    int component = -1;
    switch (ea.getKey()) {
      case 'w':
        component = MoveW;
        break;
      case 'a':
        component = MoveA;
        break;
      case 's':
        component = MoveS;
        break;
      case 'd':
        component = MoveD;
        break;
      case 'q':
        component = MoveQ;
        break;
      case 'e':
        component = MoveE;
        break;
      case 'f':
        component = MoveF;
        break;
      case 'z':
        component = MoveZ;
        break;
      default:
        return false;
    }

    mState->moveComponents[component] = down;
    return true;
  }

private:
  std::shared_ptr<AtlasPuppetState> mState;
};

//==============================================================================
void toggleEndEffectorTarget(
    AtlasPuppetState& state,
    const dart::simulation::WorldPtr& world,
    std::size_t index)
{
  if (index >= state.constraintActive.size())
    return;

  auto* ee = state.atlas->getEndEffector(state.endEffectorIndex[index]);
  const auto& ik = ee->getIK();
  if (!ik)
    return;

  if (state.constraintActive[index]) {
    state.constraintActive[index] = false;
    ik->getErrorMethod().setBounds(state.defaultBounds[index]);
    ik->getTarget()->setRelativeTransform(state.defaultTargetTf[index]);
    world->removeSimpleFrame(ik->getTarget());
  } else {
    state.constraintActive[index] = true;
    ik->getErrorMethod().setBounds();
    ik->getTarget()->setTransform(ee->getTransform());
    world->addSimpleFrame(ik->getTarget());
  }
}

//==============================================================================
void resetToRestConfig(AtlasPuppetState& state)
{
  for (std::size_t i = 0; i < state.atlas->getNumDofs(); ++i) {
    if (i < 2 || 4 < i)
      state.atlas->getDof(i)->setPosition(state.restConfig[i]);
  }
}

//==============================================================================
void printDofValues(AtlasPuppetState& state)
{
  if (!state.log)
    return;
  for (std::size_t i = 0; i < state.atlas->getNumDofs(); ++i) {
    std::ostringstream ss;
    ss << state.atlas->getDof(i)->getName() << ": "
       << state.atlas->getDof(i)->getPosition();
    state.log(ss.str());
  }
}

} // namespace

//==============================================================================
DemoScene makeAtlasPuppetScene()
{
  DemoScene scene;
  scene.id = "atlas_puppet";
  scene.title = "Atlas Puppet";
  scene.category = "Control & IK";
  scene.summary
      = "Atlas whole-body IK puppet with balance and support overlays.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();
    // Purely kinematic (see the file comment): zero gravity keeps a forced
    // step (Step button / --cycle-scenes / --headless) from making the
    // unactuated robot collapse, the same mitigation wam_ikfast (B3) uses.
    world->setGravity(Eigen::Vector3d::Zero());

    auto atlas = createAtlas();
    if (!atlas)
      throw std::runtime_error(
          "failed to load dart://sample/sdf/atlas/atlas_v3_no_head.urdf");
    world->addSkeleton(atlas);
    world->addSkeleton(createGround());

    setupStartConfiguration(atlas);
    setupEndEffectors(atlas);
    setupWholeBodySolver(atlas);

    auto state = std::make_shared<AtlasPuppetState>();
    state->atlas = atlas;
    state->restConfig = atlas->getPositions();

    atlas->eachEndEffector([&](EndEffector* ee) {
      if (const auto& ik = ee->getIK()) {
        state->defaultBounds.push_back(ik->getErrorMethod().getBounds());
        state->defaultTargetTf.push_back(
            ik->getTarget()->getRelativeTransform());
        state->constraintActive.push_back(false);
        state->endEffectorIndex.push_back(ee->getIndexInSkeleton());
      }
    });

    state->posture = std::dynamic_pointer_cast<RelaxedPosture>(
        atlas->getIK(true)->getObjective());
    state->balance
        = std::dynamic_pointer_cast<dart::constraint::BalanceConstraint>(
            atlas->getIK(true)->getProblem()->getEqConstraint(1));

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(5.34, 3.00, 2.41),
        ::osg::Vec3d(0.00, 0.00, 1.00),
        ::osg::Vec3d(-0.20, -0.08, 0.98)};

    setup.onActivate = [state, atlas](DemoHostContext& ctx) {
      auto* viewer = ctx.viewer();
      state->log = [ctx](const std::string& message) {
        ctx.log(message);
      };

      const bool resumeSimulation = viewer->isSimulating();
      viewer->allowSimulation(false);
      ctx.addTeardown([viewer, resumeSimulation] {
        viewer->allowSimulation(true);
        if (resumeSimulation)
          viewer->simulate(true);
      });

      for (std::size_t i = 0; i < atlas->getNumBodyNodes(); ++i) {
        if (auto* dnd
            = viewer->enableDragAndDrop(atlas->getBodyNode(i), false, false))
          ctx.addTeardown([viewer, dnd] { viewer->disableDragAndDrop(dnd); });
      }
      atlas->eachEndEffector([&](EndEffector* ee) {
        if (!ee->getIK())
          return;
        if (const auto& frame
            = std::dynamic_pointer_cast<dart::gui::osg::InteractiveFrame>(
                ee->getIK()->getTarget())) {
          if (auto* dnd = viewer->enableDragAndDrop(frame.get()))
            ctx.addTeardown([viewer, dnd] { viewer->disableDragAndDrop(dnd); });
        }
      });

      ctx.addAttachment(
          new dart::gui::osg::SupportPolygonVisual(atlas, kDisplayElevation));
      ctx.addEventHandler(new AtlasPuppetMoveHandler(state));
    };

    setup.keyActions.push_back(
        KeyAction{'1', "Toggle l_hand target", [state, world] {
                    toggleEndEffectorTarget(*state, world, 0);
                  }});
    setup.keyActions.push_back(
        KeyAction{'2', "Toggle r_hand target", [state, world] {
                    toggleEndEffectorTarget(*state, world, 1);
                  }});
    setup.keyActions.push_back(
        KeyAction{'3', "Toggle l_foot target", [state, world] {
                    toggleEndEffectorTarget(*state, world, 2);
                  }});
    setup.keyActions.push_back(
        KeyAction{'4', "Toggle r_foot target", [state, world] {
                    toggleEndEffectorTarget(*state, world, 3);
                  }});
    setup.keyActions.push_back(
        KeyAction{'x', "Toggle l_foot support", [atlas] {
                    auto* ee = atlas->getEndEffector("l_foot");
                    ee->getSupport()->setActive(!ee->getSupport()->isActive());
                  }});
    setup.keyActions.push_back(
        KeyAction{'c', "Toggle r_foot support", [atlas] {
                    auto* ee = atlas->getEndEffector("r_foot");
                    ee->getSupport()->setActive(!ee->getSupport()->isActive());
                  }});
    setup.keyActions.push_back(KeyAction{'t', "Reset to rest pose", [state] {
                                           resetToRestConfig(*state);
                                         }});
    setup.keyActions.push_back(KeyAction{'p', "Print DOF values", [state] {
                                           printDofValues(*state);
                                         }});

    setup.renderPanel = [state] {
      updatePuppet(*state);

      ImGui::Text(
          "Targets: l_hand %s  r_hand %s  l_foot %s  r_foot %s",
          state->constraintActive.size() > 0 && state->constraintActive[0]
              ? "on"
              : "off",
          state->constraintActive.size() > 1 && state->constraintActive[1]
              ? "on"
              : "off",
          state->constraintActive.size() > 2 && state->constraintActive[2]
              ? "on"
              : "off",
          state->constraintActive.size() > 3 && state->constraintActive[3]
              ? "on"
              : "off");
      ImGui::TextWrapped(
          "Purely kinematic (physics stepping is disabled for this scene). "
          "Alt+drag translates a body, Ctrl+drag rotates it, Shift+drag "
          "moves it via its parent joint. W/A/S/D move the robot, Q/E "
          "rotate it, F/Z change elevation; hold 'r' to optimize posture "
          "and balance. Green = support polygon, blue/red ball = center of "
          "mass, green ball = centroid.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
