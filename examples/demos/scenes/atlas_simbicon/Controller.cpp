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

#include "Controller.hpp"

#include "State.hpp"
#include "StateMachine.hpp"
#include "TerminalCondition.hpp"
#include "dart/common/Macros.hpp"

namespace dart_demos {
namespace atlas_simbicon {

//==============================================================================
Controller::Controller(
    dart::dynamics::SkeletonPtr atlasRobot,
    dart::constraint::ConstraintSolver* collisionSolver)
  : mAtlasRobot(atlasRobot),
    mConstratinSolver(collisionSolver),
    mCurrentStateMachine(nullptr),
    mPelvisHarnessOn(false),
    mLeftFootHarnessOn(false),
    mRightFootHarnessOn(false),
    mMinPelvisHeight(-0.70),
    mMaxPelvisHeight(0.30),
    mWeldJointConstraintPelvis(nullptr),
    mWeldJointConstraintLeftFoot(nullptr),
    mWeldJointConstraintRightFoot(nullptr),
    mVerbosity(false)
{
  mCoronalLeftHip = mAtlasRobot->getDof("l_leg_hpx")->getIndexInSkeleton();
  mCoronalRightHip = mAtlasRobot->getDof("r_leg_hpx")->getIndexInSkeleton();
  mSagitalLeftHip = mAtlasRobot->getDof("l_leg_hpy")->getIndexInSkeleton();
  mSagitalRightHip = mAtlasRobot->getDof("r_leg_hpy")->getIndexInSkeleton();

  _buildStateMachines();
  _setJointDamping();

  mInitialState = mAtlasRobot->getConfiguration(
      dart::dynamics::Skeleton::CONFIG_POSITIONS
      | dart::dynamics::Skeleton::CONFIG_VELOCITIES);
}

//==============================================================================
Controller::~Controller()
{
  for (auto* sm : mStateMachines)
    delete sm;
}

//==============================================================================
void Controller::update()
{
  if (isAllowingControl())
    mCurrentStateMachine->computeControlForce(mAtlasRobot->getTimeStep());
}

//==============================================================================
dart::dynamics::SkeletonPtr Controller::getAtlasRobot()
{
  return mAtlasRobot;
}

//==============================================================================
StateMachine* Controller::getCurrentState()
{
  return mCurrentStateMachine;
}

//==============================================================================
void Controller::changeStateMachine(
    StateMachine* stateMachine, double currentTime)
{
  DART_ASSERT(
      _containStateMachine(stateMachine)
      && "stateMachine should be in mStateMachines");

  if (mCurrentStateMachine == stateMachine)
    return;

  const std::string prevName = mCurrentStateMachine->getName();
  const std::string nextName = stateMachine->getName();

  mCurrentStateMachine->end(currentTime);
  mCurrentStateMachine = stateMachine;
  mCurrentStateMachine->begin(currentTime);

  if (mVerbosity) {
    dtmsg << "State machine transition: from [" << prevName << "] to ["
          << nextName << "]." << std::endl;
  }
}

//==============================================================================
void Controller::changeStateMachine(const std::string& name, double currentTime)
{
  StateMachine* stateMachine = _findStateMachine(name);
  DART_ASSERT(stateMachine != nullptr && "Invalid state machine.");
  changeStateMachine(stateMachine, currentTime);
}

//==============================================================================
void Controller::changeStateMachine(std::size_t idx, double currentTime)
{
  DART_ASSERT(idx <= mStateMachines.size() && "Invalid index of StateMachine.");
  changeStateMachine(mStateMachines[idx], currentTime);
}

//==============================================================================
bool Controller::isAllowingControl() const
{
  auto* pelvis = mAtlasRobot->getBodyNode("pelvis");
  const Eigen::Isometry3d tf = pelvis->getTransform();
  const double y = tf.translation()[1];
  return !(y < mMinPelvisHeight || mMaxPelvisHeight < y);
}

//==============================================================================
void Controller::keyboard(
    unsigned char key, int /*x*/, int /*y*/, double currentTime)
{
  switch (key) {
    case 'h':
      if (mPelvisHarnessOn)
        unharnessPelvis();
      else
        harnessPelvis();
      break;
    case 'j':
      if (mLeftFootHarnessOn)
        unharnessLeftFoot();
      else
        harnessLeftFoot();
      break;
    case 'k':
      if (mRightFootHarnessOn)
        unharnessRightFoot();
      else
        harnessRightFoot();
      break;
    case 'r':
      resetRobot();
      break;
    case 'n':
      mCurrentStateMachine->transiteToNextState(currentTime);
      break;
    case '1':
      changeStateMachine("standing", currentTime);
      break;
    case '2':
      changeStateMachine("walking in place", currentTime);
      break;
    case '3':
      changeStateMachine("walking", currentTime);
      break;
    case '4':
      changeStateMachine("running", currentTime);
      break;
    default:
      break;
  }
}

//==============================================================================
void Controller::printDebugInfo() const
{
  std::cout << "[ATLAS Robot]" << std::endl
            << " NUM NODES : " << mAtlasRobot->getNumBodyNodes() << std::endl
            << " NUM DOF   : " << mAtlasRobot->getNumDofs() << std::endl
            << " NUM JOINTS: " << mAtlasRobot->getNumBodyNodes() << std::endl;

  for (std::size_t i = 0; i < mAtlasRobot->getNumBodyNodes(); ++i) {
    auto* joint = mAtlasRobot->getJoint(i);
    auto* body = mAtlasRobot->getBodyNode(i);
    auto* parentBody = mAtlasRobot->getBodyNode(i)->getParentBodyNode();

    std::cout << "  Joint [" << i << "]: " << joint->getName() << " ("
              << joint->getNumDofs() << ")" << std::endl;
    if (parentBody != nullptr)
      std::cout << "    Parent body: " << parentBody->getName() << std::endl;
    std::cout << "    Child body : " << body->getName() << std::endl;
  }
}

//==============================================================================
void Controller::harnessPelvis()
{
  if (mPelvisHarnessOn)
    return;

  auto* bd = mAtlasRobot->getBodyNode("pelvis");
  mWeldJointConstraintPelvis
      = std::make_shared<dart::constraint::WeldJointConstraint>(bd);
  mConstratinSolver->addConstraint(mWeldJointConstraintPelvis);
  mPelvisHarnessOn = true;

  if (mVerbosity)
    dtmsg << "Pelvis is harnessed." << std::endl;
}

//==============================================================================
void Controller::unharnessPelvis()
{
  if (!mPelvisHarnessOn)
    return;

  mConstratinSolver->removeConstraint(mWeldJointConstraintPelvis);
  mPelvisHarnessOn = false;

  if (mVerbosity)
    dtmsg << "Pelvis is unharnessed." << std::endl;
}

//==============================================================================
void Controller::harnessLeftFoot()
{
  if (mLeftFootHarnessOn)
    return;

  auto* bd = mAtlasRobot->getBodyNode("l_foot");
  mWeldJointConstraintLeftFoot
      = std::make_shared<dart::constraint::WeldJointConstraint>(bd);
  mLeftFootHarnessOn = true;

  if (mVerbosity)
    dtmsg << "Left foot is harnessed." << std::endl;
}

//==============================================================================
void Controller::unharnessLeftFoot()
{
  if (!mLeftFootHarnessOn)
    return;

  mConstratinSolver->removeConstraint(mWeldJointConstraintLeftFoot);
  mLeftFootHarnessOn = false;

  if (mVerbosity)
    dtmsg << "Left foot is unharnessed." << std::endl;
}

//==============================================================================
void Controller::harnessRightFoot()
{
  if (mRightFootHarnessOn)
    return;

  auto* bd = mAtlasRobot->getBodyNode("r_foot");
  mWeldJointConstraintRightFoot
      = std::make_shared<dart::constraint::WeldJointConstraint>(bd);
  mRightFootHarnessOn = true;

  if (mVerbosity)
    dtmsg << "Right foot is harnessed." << std::endl;
}

//==============================================================================
void Controller::unharnessRightFoot()
{
  if (!mRightFootHarnessOn)
    return;

  mConstratinSolver->removeConstraint(mWeldJointConstraintRightFoot);
  mRightFootHarnessOn = false;

  if (mVerbosity)
    dtmsg << "Right foot is unharnessed." << std::endl;
}

//==============================================================================
void Controller::resetRobot()
{
  mAtlasRobot->setConfiguration(mInitialState);

  if (mVerbosity)
    dtmsg << "Robot is reset." << std::endl;
}

//==============================================================================
void Controller::setVerbosity(bool verbosity)
{
  mVerbosity = verbosity;
}

//==============================================================================
void Controller::_buildStateMachines()
{
  mStateMachines.push_back(_createStandingStateMachine());
  mStateMachines.push_back(_createWalkingInPlaceStateMachine());
  mStateMachines.push_back(_createWalkingStateMachine());
  mStateMachines.push_back(_createRunningStateMachine());

  // Default controller: walking in place.
  mCurrentStateMachine = mStateMachines[1];
  mCurrentStateMachine->begin(0.0);
}

//==============================================================================
StateMachine* Controller::_createStandingStateMachine()
{
  using namespace dart::math::suffixes;

  auto* standing = new StateMachine("standing");
  auto* standingState0 = new State(mAtlasRobot, "0");
  auto* tcStanding0 = new TimerCondition(standingState0, 0.3);

  standingState0->setTerminalCondition(tcStanding0);
  standingState0->setNextState(standingState0);

  standingState0->setDesiredJointPosition("back_bky", 15.00_deg);
  standingState0->setDesiredJointPosition("l_leg_hpy", -10.00_deg);
  standingState0->setDesiredJointPosition("r_leg_hpy", -10.00_deg);
  standingState0->setDesiredJointPosition("l_leg_kny", 30.00_deg);
  standingState0->setDesiredJointPosition("r_leg_kny", 30.00_deg);
  standingState0->setDesiredJointPosition("l_leg_aky", -16.80_deg);
  standingState0->setDesiredJointPosition("r_leg_aky", -16.80_deg);
  standingState0->setDesiredJointPosition("l_arm_shx", -90.0_deg);
  standingState0->setDesiredJointPosition("r_arm_shx", +90.0_deg);

  standing->addState(standingState0);
  standing->setInitialState(standingState0);

  return standing;
}

//==============================================================================
StateMachine* Controller::_createWalkingInPlaceStateMachine()
{
  using namespace dart::math::suffixes;

  const double cd = 0.5;
  const double cv = 0.2;
  const double pelvis = -4.75_deg;

  const double swh02 = 0.50;
  const double swk02 = -1.10;
  const double swa02 = 0.60;
  const double stk02 = -0.05;
  const double sta02 = 0.00;

  const double swh13 = -0.10;
  const double swk13 = -0.05;
  const double swa13 = 0.15;
  const double stk13 = -0.10;
  const double sta13 = 0.00;

  auto* sm = new StateMachine("walking in place");

  auto* state0 = new State(mAtlasRobot, "0");
  auto* state1 = new State(mAtlasRobot, "1");
  auto* state2 = new State(mAtlasRobot, "2");
  auto* state3 = new State(mAtlasRobot, "3");

  auto* cond0 = new TimerCondition(state0, 0.3);
  auto* cond1 = new BodyContactCondition(state1, _getRightFoot());
  auto* cond2 = new TimerCondition(state2, 0.3);
  auto* cond3 = new BodyContactCondition(state3, _getLeftFoot());

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);
  state2->setTerminalCondition(cond2);
  state3->setTerminalCondition(cond3);

  state0->setNextState(state1);
  state1->setNextState(state2);
  state2->setNextState(state3);
  state3->setNextState(state0);

  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToLeftFoot();
  state2->setStanceFootToRightFoot();
  state3->setStanceFootToRightFoot();

  state0->setDesiredPelvisGlobalAngleOnSagital(0.0_deg);
  state1->setDesiredPelvisGlobalAngleOnSagital(0.0_deg);
  state2->setDesiredPelvisGlobalAngleOnSagital(0.0_deg);
  state3->setDesiredPelvisGlobalAngleOnSagital(0.0_deg);
  state0->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state1->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state2->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state3->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);

  state0->setDesiredJointPosition("back_bky", -pelvis);
  state0->setDesiredJointPosition("r_leg_hpy", -swh02);
  state0->setDesiredJointPosition("r_leg_kny", -swk02);
  state0->setDesiredJointPosition("r_leg_aky", -swa02);
  state0->setDesiredJointPosition("l_leg_kny", -stk02);
  state0->setDesiredJointPosition("l_leg_aky", -sta02);
  state0->setDesiredJointPosition("l_arm_shy", -20.00_deg);
  state0->setDesiredJointPosition("r_arm_shy", +10.00_deg);
  state0->setDesiredJointPosition("l_arm_shx", -80.00_deg);
  state0->setDesiredJointPosition("r_arm_shx", +80.00_deg);
  state0->setFeedbackCoronalCOMDistance(mCoronalLeftHip, -cd);
  state0->setFeedbackCoronalCOMVelocity(mCoronalLeftHip, -cv);
  state0->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);
  state0->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);
  state0->setFeedbackSagitalCOMDistance(mSagitalLeftHip, -cd);
  state0->setFeedbackSagitalCOMVelocity(mSagitalLeftHip, -cv);
  state0->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd);
  state0->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv);

  state1->setDesiredJointPosition("back_bky", -pelvis);
  state1->setDesiredJointPosition("l_leg_hpy", -swh13);
  state1->setDesiredJointPosition("l_leg_kny", -swk13);
  state1->setDesiredJointPosition("l_leg_aky", -swa13);
  state1->setDesiredJointPosition("r_leg_kny", -stk13);
  state1->setDesiredJointPosition("r_leg_aky", -sta13);
  state1->setDesiredJointPosition("l_arm_shy", +10.00_deg);
  state1->setDesiredJointPosition("r_arm_shy", -20.00_deg);
  state1->setDesiredJointPosition("l_arm_shx", -80.00_deg);
  state1->setDesiredJointPosition("r_arm_shx", +80.00_deg);
  state1->setFeedbackCoronalCOMDistance(mCoronalLeftHip, -cd);
  state1->setFeedbackCoronalCOMVelocity(mCoronalLeftHip, -cv);
  state1->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);
  state1->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);
  state1->setFeedbackSagitalCOMDistance(mSagitalLeftHip, -cd);
  state1->setFeedbackSagitalCOMVelocity(mSagitalLeftHip, -cv);
  state1->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd);
  state1->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv);

  state2->setDesiredJointPosition("back_bky", -pelvis);
  state2->setDesiredJointPosition("l_leg_hpy", -swh02);
  state2->setDesiredJointPosition("l_leg_kny", -swk02);
  state2->setDesiredJointPosition("l_leg_aky", -swa02);
  state2->setDesiredJointPosition("r_leg_kny", -stk02);
  state2->setDesiredJointPosition("r_leg_aky", -sta02);
  state2->setDesiredJointPosition("l_arm_shy", +10.00_deg);
  state2->setDesiredJointPosition("r_arm_shy", -20.00_deg);
  state2->setDesiredJointPosition("l_arm_shx", -80.00_deg);
  state2->setDesiredJointPosition("r_arm_shx", +80.00_deg);
  state2->setFeedbackCoronalCOMDistance(mCoronalLeftHip, -cd);
  state2->setFeedbackCoronalCOMVelocity(mCoronalLeftHip, -cv);
  state2->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);
  state2->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);
  state2->setFeedbackSagitalCOMDistance(mSagitalLeftHip, -cd);
  state2->setFeedbackSagitalCOMVelocity(mSagitalLeftHip, -cv);
  state2->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd);
  state2->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv);

  state3->setDesiredJointPosition("back_bky", -pelvis);
  state3->setDesiredJointPosition("r_leg_hpy", -swh13);
  state3->setDesiredJointPosition("r_leg_kny", -swk13);
  state3->setDesiredJointPosition("r_leg_aky", -swa13);
  state3->setDesiredJointPosition("l_leg_kny", -stk13);
  state3->setDesiredJointPosition("l_leg_aky", -sta13);
  state3->setDesiredJointPosition("l_arm_shy", -20.00_deg);
  state3->setDesiredJointPosition("r_arm_shy", +10.00_deg);
  state3->setDesiredJointPosition("l_arm_shx", -80.00_deg);
  state3->setDesiredJointPosition("r_arm_shx", +80.00_deg);
  state3->setFeedbackCoronalCOMDistance(mCoronalLeftHip, -cd);
  state3->setFeedbackCoronalCOMVelocity(mCoronalLeftHip, -cv);
  state3->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);
  state3->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);
  state3->setFeedbackSagitalCOMDistance(mSagitalLeftHip, -cd);
  state3->setFeedbackSagitalCOMVelocity(mSagitalLeftHip, -cv);
  state3->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd);
  state3->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv);

  sm->addState(state0);
  sm->addState(state1);
  sm->addState(state2);
  sm->addState(state3);
  sm->setInitialState(state1);

  return sm;
}

//==============================================================================
StateMachine* Controller::_createWalkingStateMachine()
{
  using namespace dart::math::suffixes;

  const double cd = 0.5;
  const double cv = 0.2;
  const double pelvis = -10.0_deg;

  const double swh02 = 0.50;
  const double swk02 = -1.10;
  const double swa02 = 0.60;
  const double stk02 = -0.05;
  const double sta02 = 0.00;

  const double swh13 = -0.10;
  const double swk13 = -0.05;
  const double swa13 = 0.15;
  const double stk13 = -0.10;
  const double sta13 = 0.00;

  auto* sm = new StateMachine("walking");

  auto* state0 = new State(mAtlasRobot, "0");
  auto* state1 = new State(mAtlasRobot, "1");
  auto* state2 = new State(mAtlasRobot, "2");
  auto* state3 = new State(mAtlasRobot, "3");

  auto* cond0 = new TimerCondition(state0, 0.3);
  auto* cond1 = new BodyContactCondition(state1, _getRightFoot());
  auto* cond2 = new TimerCondition(state2, 0.3);
  auto* cond3 = new BodyContactCondition(state3, _getLeftFoot());

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);
  state2->setTerminalCondition(cond2);
  state3->setTerminalCondition(cond3);

  state0->setNextState(state1);
  state1->setNextState(state2);
  state2->setNextState(state3);
  state3->setNextState(state0);

  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToLeftFoot();
  state2->setStanceFootToRightFoot();
  state3->setStanceFootToRightFoot();

  state0->setDesiredPelvisGlobalAngleOnSagital(0.0_deg);
  state1->setDesiredPelvisGlobalAngleOnSagital(0.0_deg);
  state2->setDesiredPelvisGlobalAngleOnSagital(0.0_deg);
  state3->setDesiredPelvisGlobalAngleOnSagital(0.0_deg);
  state0->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state1->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state2->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state3->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);

  state0->setDesiredJointPosition("back_bky", -pelvis);
  state0->setDesiredJointPosition("r_leg_hpy", -swh02);
  state0->setDesiredJointPosition("r_leg_kny", -swk02);
  state0->setDesiredJointPosition("r_leg_aky", -swa02);
  state0->setDesiredJointPosition("l_leg_kny", -stk02);
  state0->setDesiredJointPosition("l_leg_aky", -sta02);
  state0->setDesiredJointPosition("l_arm_shy", -20.00_deg);
  state0->setDesiredJointPosition("r_arm_shy", +10.00_deg);
  state0->setDesiredJointPosition("l_arm_shx", -80.00_deg);
  state0->setDesiredJointPosition("r_arm_shx", +80.00_deg);
  state0->setFeedbackCoronalCOMDistance(mCoronalLeftHip, -cd);
  state0->setFeedbackCoronalCOMVelocity(mCoronalLeftHip, -cv);
  state0->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);
  state0->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);
  state0->setFeedbackSagitalCOMDistance(mSagitalLeftHip, -cd);
  state0->setFeedbackSagitalCOMVelocity(mSagitalLeftHip, -cv);
  state0->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd);
  state0->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv);

  state1->setDesiredJointPosition("back_bky", -pelvis);
  state1->setDesiredJointPosition("l_leg_hpy", -swh13);
  state1->setDesiredJointPosition("l_leg_kny", -swk13);
  state1->setDesiredJointPosition("l_leg_aky", -swa13);
  state1->setDesiredJointPosition("r_leg_kny", -stk13);
  state1->setDesiredJointPosition("r_leg_aky", -sta13);
  state1->setDesiredJointPosition("l_arm_shy", +10.00_deg);
  state1->setDesiredJointPosition("r_arm_shy", -20.00_deg);
  state1->setDesiredJointPosition("l_arm_shx", -80.00_deg);
  state1->setDesiredJointPosition("r_arm_shx", +80.00_deg);
  state1->setFeedbackCoronalCOMDistance(mCoronalLeftHip, -cd);
  state1->setFeedbackCoronalCOMVelocity(mCoronalLeftHip, -cv);
  state1->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);
  state1->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);
  state1->setFeedbackSagitalCOMDistance(mSagitalLeftHip, -cd);
  state1->setFeedbackSagitalCOMVelocity(mSagitalLeftHip, -cv);
  state1->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd);
  state1->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv);

  state2->setDesiredJointPosition("back_bky", -pelvis);
  state2->setDesiredJointPosition("l_leg_hpy", -swh02);
  state2->setDesiredJointPosition("l_leg_kny", -swk02);
  state2->setDesiredJointPosition("l_leg_aky", -swa02);
  state2->setDesiredJointPosition("r_leg_kny", -stk02);
  state2->setDesiredJointPosition("r_leg_aky", -sta02);
  state2->setDesiredJointPosition("l_arm_shy", +10.00_deg);
  state2->setDesiredJointPosition("r_arm_shy", -20.00_deg);
  state2->setDesiredJointPosition("l_arm_shx", -80.00_deg);
  state2->setDesiredJointPosition("r_arm_shx", +80.00_deg);
  state2->setFeedbackCoronalCOMDistance(mCoronalLeftHip, -cd);
  state2->setFeedbackCoronalCOMVelocity(mCoronalLeftHip, -cv);
  state2->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);
  state2->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);
  state2->setFeedbackSagitalCOMDistance(mSagitalLeftHip, -cd);
  state2->setFeedbackSagitalCOMVelocity(mSagitalLeftHip, -cv);
  state2->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd);
  state2->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv);

  state3->setDesiredJointPosition("back_bky", -pelvis);
  state3->setDesiredJointPosition("r_leg_hpy", -swh13);
  state3->setDesiredJointPosition("r_leg_kny", -swk13);
  state3->setDesiredJointPosition("r_leg_aky", -swa13);
  state3->setDesiredJointPosition("l_leg_kny", -stk13);
  state3->setDesiredJointPosition("l_leg_aky", -sta13);
  state3->setDesiredJointPosition("l_arm_shy", -20.00_deg);
  state3->setDesiredJointPosition("r_arm_shy", +10.00_deg);
  state3->setDesiredJointPosition("l_arm_shx", -80.00_deg);
  state3->setDesiredJointPosition("r_arm_shx", +80.00_deg);
  state3->setFeedbackCoronalCOMDistance(mCoronalLeftHip, -cd);
  state3->setFeedbackCoronalCOMVelocity(mCoronalLeftHip, -cv);
  state3->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);
  state3->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);
  state3->setFeedbackSagitalCOMDistance(mSagitalLeftHip, -cd);
  state3->setFeedbackSagitalCOMVelocity(mSagitalLeftHip, -cv);
  state3->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd);
  state3->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv);

  sm->addState(state0);
  sm->addState(state1);
  sm->addState(state2);
  sm->addState(state3);
  sm->setInitialState(state1);

  return sm;
}

//==============================================================================
StateMachine* Controller::_createRunningStateMachine()
{
  using namespace dart::math::suffixes;

  const double cd = 0.5;
  const double cv = 0.2;
  const double pelvis = -10.0_deg;

  const double swh01 = 0.50;
  const double swk01 = -1.10;
  const double swa01 = 0.60;
  const double stk01 = -0.05;
  const double sta01 = 0.00;

  auto* sm = new StateMachine("running");

  auto* state0 = new State(mAtlasRobot, "0");
  auto* state1 = new State(mAtlasRobot, "1");

  auto* cond0 = new TimerCondition(state0, 0.15);
  auto* cond1 = new TimerCondition(state1, 0.15);

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);

  state0->setNextState(state1);
  state1->setNextState(state0);

  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToRightFoot();

  state0->setDesiredPelvisGlobalAngleOnSagital(0.0_deg);
  state1->setDesiredPelvisGlobalAngleOnSagital(0.0_deg);
  state0->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state1->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);

  state0->setDesiredJointPosition("back_bky", -pelvis);
  state0->setDesiredJointPosition("r_leg_hpy", -swh01);
  state0->setDesiredJointPosition("r_leg_kny", -swk01);
  state0->setDesiredJointPosition("r_leg_aky", -swa01);
  state0->setDesiredJointPosition("l_leg_kny", -stk01);
  state0->setDesiredJointPosition("l_leg_aky", -sta01);
  state0->setDesiredJointPosition("l_arm_shy", -45.00_deg);
  state0->setDesiredJointPosition("r_arm_shy", +15.00_deg);
  state0->setDesiredJointPosition("l_arm_shx", -80.00_deg);
  state0->setDesiredJointPosition("r_arm_shx", +80.00_deg);
  state0->setFeedbackCoronalCOMDistance(mCoronalLeftHip, -cd);
  state0->setFeedbackCoronalCOMVelocity(mCoronalLeftHip, -cv);
  state0->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);
  state0->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);
  state0->setFeedbackSagitalCOMDistance(mSagitalLeftHip, -cd);
  state0->setFeedbackSagitalCOMVelocity(mSagitalLeftHip, -cv);
  state0->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd);
  state0->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv);

  state1->setDesiredJointPosition("back_bky", -pelvis);
  state1->setDesiredJointPosition("l_leg_hpy", -swh01);
  state1->setDesiredJointPosition("l_leg_kny", -swk01);
  state1->setDesiredJointPosition("l_leg_aky", -swa01);
  state1->setDesiredJointPosition("r_leg_kny", -stk01);
  state1->setDesiredJointPosition("r_leg_aky", -sta01);
  state1->setDesiredJointPosition("l_arm_shy", +15.00_deg);
  state1->setDesiredJointPosition("r_arm_shy", -45.00_deg);
  state1->setDesiredJointPosition("l_arm_shx", -80.00_deg);
  state1->setDesiredJointPosition("r_arm_shx", +80.00_deg);
  state1->setFeedbackCoronalCOMDistance(mCoronalLeftHip, -cd);
  state1->setFeedbackCoronalCOMVelocity(mCoronalLeftHip, -cv);
  state1->setFeedbackCoronalCOMDistance(mCoronalRightHip, -cd);
  state1->setFeedbackCoronalCOMVelocity(mCoronalRightHip, -cv);
  state1->setFeedbackSagitalCOMDistance(mSagitalLeftHip, -cd);
  state1->setFeedbackSagitalCOMVelocity(mSagitalLeftHip, -cv);
  state1->setFeedbackSagitalCOMDistance(mSagitalRightHip, -cd);
  state1->setFeedbackSagitalCOMVelocity(mSagitalRightHip, -cv);

  sm->addState(state0);
  sm->addState(state1);
  sm->setInitialState(state0);

  return sm;
}

//==============================================================================
void Controller::_setJointDamping()
{
  for (std::size_t i = 1; i < mAtlasRobot->getNumBodyNodes(); ++i) {
    auto* joint = mAtlasRobot->getJoint(i);
    for (std::size_t j = 0; j < joint->getNumDofs(); ++j)
      joint->setDampingCoefficient(j, 80.0);
  }
}

//==============================================================================
dart::dynamics::BodyNode* Controller::_getLeftFoot() const
{
  return mAtlasRobot->getBodyNode("l_foot");
}

//==============================================================================
dart::dynamics::BodyNode* Controller::_getRightFoot() const
{
  return mAtlasRobot->getBodyNode("r_foot");
}

//==============================================================================
bool Controller::_containStateMachine(const StateMachine* stateMachine) const
{
  for (const auto* sm : mStateMachines) {
    if (sm == stateMachine)
      return true;
  }
  return false;
}

//==============================================================================
bool Controller::_containStateMachine(const std::string& name) const
{
  return _containStateMachine(_findStateMachine(name));
}

//==============================================================================
StateMachine* Controller::_findStateMachine(const std::string& name) const
{
  for (auto* sm : mStateMachines) {
    if (sm->getName() == name)
      return sm;
  }
  return nullptr;
}

} // namespace atlas_simbicon
} // namespace dart_demos
