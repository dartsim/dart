/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "apps/atlasRobot/Controller.h"

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/Joint.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/constraint/WeldJointConstraint.h"
#include "dart/collision/CollisionDetector.h"

#include "apps/atlasRobot/State.h"
#include "apps/atlasRobot/StateMachine.h"
#include "apps/atlasRobot/TerminalCondition.h"

using namespace std;

using namespace Eigen;

using namespace dart;
using namespace constraint;
using namespace dynamics;

//==============================================================================
Controller::Controller(Skeleton* _atlasRobot,
                       ConstraintSolver* _collisionSolver)
  : mAtlasRobot(_atlasRobot),
    mConstratinSolver(_collisionSolver),
    mCurrentStateMachine(NULL),
    mPelvisHarnessOn(false),
    mLeftFootHarnessOn(false),
    mRightFootHarnessOn(false),
    mWeldJointConstraintPelvis(NULL),
    mWeldJointConstraintLeftFoot(NULL),
    mWeldJointConstraintRightFoot(NULL)
{
  _buildStateMachines();
  _setJointDamping();

//  harnessPelvis();
//  harnessLeftFoot();
//  harnessRightFoot();

  mInitialState = mAtlasRobot->getState();
}

//==============================================================================
Controller::~Controller()
{
  for (vector<StateMachine*>::iterator it = mStateMachines.begin();
       it != mStateMachines.end(); ++it)
  {
    delete *it;
  }
}

//==============================================================================
void Controller::update(double _currentTime)
{
  // Compute control force
  mCurrentStateMachine->computeControlForce(mAtlasRobot->getTimeStep());
}

//==============================================================================
Skeleton* Controller::getAtlasRobot()
{
  return mAtlasRobot;
}

StateMachine*Controller::getCurrentState()
{
  return mCurrentStateMachine;
}

//==============================================================================
void Controller::changeStateMachine(StateMachine* _stateMachine,
                                    double _currentTime)
{
  assert(_containStateMachine(_stateMachine)
         && "_stateMachine should be in mStateMachines");

  if (mCurrentStateMachine == _stateMachine)
  {
    return;
  }

  string prevName = mCurrentStateMachine->getName();
  string nextName = _stateMachine->getName();

  // Finish current state
  mCurrentStateMachine->end(_currentTime);

  // Transite to _state
  mCurrentStateMachine = _stateMachine;
  mCurrentStateMachine->begin(_currentTime);

  dtmsg << "State machine transition: from [" << prevName << "] to ["
        << nextName << "]." << endl;
}

//==============================================================================
void Controller::changeStateMachine(const string& _name, double _currentTime)
{
  // _state should be in mStates
  StateMachine* stateMachine = _findStateMachine(_name);

  assert(stateMachine != NULL && "Invaild state machine.");

  changeStateMachine(stateMachine, _currentTime);
}

//==============================================================================
void Controller::changeStateMachine(size_t _idx, double _currentTime)
{
  assert(0 <= _idx && _idx <= mStateMachines.size()
         && "Invalid index of StateMachine.");

  changeStateMachine(mStateMachines[_idx], _currentTime);
}

//==============================================================================
void Controller::keyboard(unsigned char _key, int _x, int _y,
                          double _currentTime)
{
  switch (_key)
  {
    case 'h':  // Harness pelvis toggle
      if (mPelvisHarnessOn)
        unharnessPelvis();
      else
        harnessPelvis();
      break;
    case 'j':  // Harness left foot toggle
      if (mLeftFootHarnessOn)
        unharnessLeftFoot();
      else
        harnessLeftFoot();
      break;
    case 'k':  // Harness right foot toggle
      if (mRightFootHarnessOn)
        unharnessRightFoot();
      else
        harnessRightFoot();
      break;
    case 'r':  // Reset robot
      resetRobot();
      break;
    case 'n':  // Transite to the next state manually
      mCurrentStateMachine->transiteToNextState(_currentTime);
      break;
    case '1':  // Standing controller
      changeStateMachine("standing", _currentTime);
      break;
    case '2':  // Walking in place controller
      changeStateMachine("walking in place", _currentTime);
      break;
    case '3':
      changeStateMachine("walking", _currentTime);
      break;
    case '4':
      changeStateMachine("running", _currentTime);
      break;

    default:
      break;
  }
}

//==============================================================================
void Controller::printDebugInfo() const
{
  std::cout << "[ATLAS Robot]"  << std::endl
            << " NUM NODES : " << mAtlasRobot->getNumBodyNodes() << std::endl
            << " NUM DOF   : " << mAtlasRobot->getDof() << std::endl
            << " NUM JOINTS: " << mAtlasRobot->getNumBodyNodes() << std::endl;

  for(size_t i = 0; i < mAtlasRobot->getNumBodyNodes(); ++i)
  {
    Joint* joint = mAtlasRobot->getJoint(i);
    BodyNode* body = mAtlasRobot->getBodyNode(i);
    BodyNode* parentBody = mAtlasRobot->getBodyNode(i)->getParentBodyNode();

    std::cout << "  Joint [" << i << "]: "
              << joint->getName()
              << " (" << joint->getDof() << ")"
              << std::endl;
    if (parentBody != NULL)
    {
      std::cout << "    Parent body: " << parentBody->getName() << std::endl;
    }

    std::cout << "    Child body : " << body->getName() << std::endl;
  }
}

//==============================================================================
void Controller::harnessPelvis()
{
  if (mPelvisHarnessOn)
    return;

  BodyNode* bd = mAtlasRobot->getBodyNode("pelvis");
  mWeldJointConstraintPelvis = new WeldJointConstraint(bd);
  mConstratinSolver->addConstraint(mWeldJointConstraintPelvis);
  mPelvisHarnessOn = true;

  dtmsg << "Pelvis is harnessed." << std::endl;
}

//==============================================================================
void Controller::unharnessPelvis()
{
  if (!mPelvisHarnessOn)
    return;

  mConstratinSolver->removeConstraint(mWeldJointConstraintPelvis);
  mPelvisHarnessOn = false;

  dtmsg << "Pelvis is unharnessed." << std::endl;
}

//==============================================================================
void Controller::harnessLeftFoot()
{
  if (mLeftFootHarnessOn)
    return;

  BodyNode* bd = mAtlasRobot->getBodyNode("l_foot");
  mWeldJointConstraintLeftFoot = new WeldJointConstraint(bd);
  mLeftFootHarnessOn = true;

  dtmsg << "Left foot is harnessed." << std::endl;
}

//==============================================================================
void Controller::unharnessLeftFoot()
{
  if (!mLeftFootHarnessOn)
    return;

  mConstratinSolver->removeConstraint(mWeldJointConstraintLeftFoot);
  mLeftFootHarnessOn = false;

  dtmsg << "Left foot is unharnessed." << std::endl;
}

//==============================================================================
void Controller::harnessRightFoot()
{
  if (mRightFootHarnessOn)
    return;

  BodyNode* bd = mAtlasRobot->getBodyNode("r_foot");
  mWeldJointConstraintRightFoot = new WeldJointConstraint(bd);
  mRightFootHarnessOn = true;

  dtmsg << "Right foot is harnessed." << std::endl;
}

//==============================================================================
void Controller::unharnessRightFoot()
{
  if (!mRightFootHarnessOn)
    return;

  mConstratinSolver->removeConstraint(mWeldJointConstraintRightFoot);
  mRightFootHarnessOn = false;

  dtmsg << "Right foot is unharnessed." << std::endl;
}

//==============================================================================
void Controller::resetRobot()
{
  mAtlasRobot->setState(mInitialState);
  mAtlasRobot->computeForwardKinematics(true, true, false);

  dtmsg << "Robot is reset." << std::endl;
}

//==============================================================================
void Controller::_buildStateMachines()
{
  // Standing controller
  mStateMachines.push_back(_createStandingStateMachine());

  // Walking in place controller
  mStateMachines.push_back(_createWalkingInPlaceStateMachine());

  // Walking controller
  mStateMachines.push_back(_createWalkingStateMachine());

  // Walking controller
  mStateMachines.push_back(_createRunningStateMachine());

  // Set initial (default) controller
  mCurrentStateMachine = mStateMachines[1];  // Standing controller

  // Begin the default controller
  mCurrentStateMachine->begin(0.0);
}

//==============================================================================
StateMachine* Controller::_createStandingStateMachine()
{
  StateMachine* standing = new StateMachine("standing");

  State* standingState0 = new State(mAtlasRobot, "0");

  TerminalCondition* tcStanding0 = new TimerCondition(standingState0, 0.3);

  standingState0->setTerminalCondition(tcStanding0);

  standingState0->setNextState(standingState0);

  standingState0->setDesiredJointPosition( 9, DART_RADIAN *  15.00); // angle b/w pelvis and torso
  standingState0->setDesiredJointPosition(13, DART_RADIAN * -10.00);
  standingState0->setDesiredJointPosition(14, DART_RADIAN * -10.00);
  standingState0->setDesiredJointPosition(17, DART_RADIAN *  30.00); // left knee
  standingState0->setDesiredJointPosition(18, DART_RADIAN *  30.00); // right knee
  standingState0->setDesiredJointPosition(21, DART_RADIAN * -16.80); // left ankle
  standingState0->setDesiredJointPosition(22, DART_RADIAN * -16.80); // right ankle

  standingState0->setDesiredJointPosition(19, DART_RADIAN * -90.0); // right ankle
  standingState0->setDesiredJointPosition(20, DART_RADIAN * +90.0); // right ankle

  standing->addState(standingState0);

  standing->setInitialState(standingState0);

  return standing;
}

//==============================================================================
StateMachine* Controller::_createWalkingInPlaceStateMachine()
{
  const double cd = 0.5;
  const double cv = 0.2;

  const double pelvis = DART_RADIAN * -4.75;  // angle b/w pelvis and torso

  const double swh02  =  0.50;  // swing hip
  const double swk02  = -1.10;  // swing knee
  const double swa02  =  0.60;  // swing angle
  const double stk02  = -0.05;  // stance knee
  const double sta02  =  0.00;  // stance ankle

  const double swh13  = -0.10;  // swing hip
  const double swk13  = -0.05;  // swing knee
  const double swa13  =  0.15;  // swing angle
  const double stk13  = -0.10;  // stance knee
  const double sta13  =  0.00;  // stance ankle

  StateMachine* sm = new StateMachine("walking in place");

  State* state0 = new State(mAtlasRobot, "0");
  State* state1 = new State(mAtlasRobot, "1");
  State* state2 = new State(mAtlasRobot, "2");
  State* state3 = new State(mAtlasRobot, "3");

  TerminalCondition* cond0 = new TimerCondition(state0, 0.3);
  TerminalCondition* cond1 = new BodyContactCondition(state1, _getRightFoot());
  TerminalCondition* cond2 = new TimerCondition(state2, 0.3);
  TerminalCondition* cond3 = new BodyContactCondition(state3, _getLeftFoot());

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);
  state2->setTerminalCondition(cond2);
  state3->setTerminalCondition(cond3);

  state0->setNextState(state1);
  state1->setNextState(state2);
  state2->setNextState(state3);
  state3->setNextState(state0);

  // Set stance foot
  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToLeftFoot();
  state2->setStanceFootToRightFoot();
  state3->setStanceFootToRightFoot();

  // Set global desired pelvis angle
  state0->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state1->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state2->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state3->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state0->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);
  state1->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);
  state2->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);
  state3->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);

  // Set desired joint position
  //-- State 0
  //---- pelvis
  state0->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state0->setDesiredJointPosition(14, -swh02); // right hip
  state0->setDesiredJointPosition(18, -swk02); // right knee
  state0->setDesiredJointPosition(22, -swa02); // right ankle
  //---- stance leg
  state0->setDesiredJointPosition(17, -stk02); // left knee
  state0->setDesiredJointPosition(21, -sta02); // left ankle
  //---- arm
  state0->setDesiredJointPosition(15, DART_RADIAN * -20.00); // left arm
  state0->setDesiredJointPosition(16, DART_RADIAN * +10.00); // right arm
  state0->setDesiredJointPosition(19, DART_RADIAN * -80.00); // left arm
  state0->setDesiredJointPosition(20, DART_RADIAN * +80.00); // right arm
  //---- feedback gain for hip joints
  state0->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state0->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state0->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state0->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state0->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state0->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state0->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state0->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 1
  //---- pelvis
  state1->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state1->setDesiredJointPosition(13, -swh13); // left hip
  state1->setDesiredJointPosition(17, -swk13); // left knee
  state1->setDesiredJointPosition(21, -swa13); // left ankle
  //---- stance leg
  state1->setDesiredJointPosition(18, -stk13); // right knee
  state1->setDesiredJointPosition(22, -sta13); // right ankle
  //---- arm
  state1->setDesiredJointPosition(15, DART_RADIAN * +10.00); // left arm
  state1->setDesiredJointPosition(16, DART_RADIAN * -20.00); // right arm
  state1->setDesiredJointPosition(19, DART_RADIAN * -80.00); // left arm
  state1->setDesiredJointPosition(20, DART_RADIAN * +80.00); // right arm
  //---- feedback gain for hip joints
  state1->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state1->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state1->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state1->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state1->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state1->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state1->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state1->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 2
  //---- pelvis
  state2->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state2->setDesiredJointPosition(13, -swh02); // left hip
  state2->setDesiredJointPosition(17, -swk02); // left knee
  state2->setDesiredJointPosition(21, -swa02); // left ankle
  //---- stance leg
  state2->setDesiredJointPosition(18, -stk02); // right knee
  state2->setDesiredJointPosition(22, -sta02); // right ankle
  //---- arm
  state2->setDesiredJointPosition(15, DART_RADIAN * +10.00); // left arm
  state2->setDesiredJointPosition(16, DART_RADIAN * -20.00); // right arm
  state2->setDesiredJointPosition(19, DART_RADIAN * -80.00); // left arm
  state2->setDesiredJointPosition(20, DART_RADIAN * +80.00); // right arm
  //---- feedback gain for hip joints
  state2->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state2->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state2->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state2->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state2->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state2->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state2->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state2->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 3
  //---- pelvis
  state3->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state3->setDesiredJointPosition(14, -swh13); // right hip
  state3->setDesiredJointPosition(18, -swk13); // right knee
  state3->setDesiredJointPosition(22, -swa13); // right ankle
  //---- stance leg
  state3->setDesiredJointPosition(17, -stk13); // left knee
  state3->setDesiredJointPosition(21, -sta13); // left ankle
  //---- arm
  state3->setDesiredJointPosition(15, DART_RADIAN * -20.00); // left arm
  state3->setDesiredJointPosition(16, DART_RADIAN * +10.00); // right arm
  state3->setDesiredJointPosition(19, DART_RADIAN * -80.00); // left arm
  state3->setDesiredJointPosition(20, DART_RADIAN * +80.00); // right arm
  //---- feedback gain for hip joints
  state3->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state3->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state3->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state3->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state3->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state3->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state3->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state3->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

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
  const double cd = 0.5;
  const double cv = 0.2;

  const double pelvis = DART_RADIAN * -10.0;  // angle b/w pelvis and torso

  const double swh02  =  0.50;  // swing hip
  const double swk02  = -1.10;  // swing knee
  const double swa02  =  0.60;  // swing angle
  const double stk02  = -0.05;  // stance knee
  const double sta02  =  0.00;  // stance ankle

  const double swh13  = -0.10;  // swing hip
  const double swk13  = -0.05;  // swing knee
  const double swa13  =  0.15;  // swing angle
  const double stk13  = -0.10;  // stance knee
  const double sta13  =  0.00;  // stance ankle

  StateMachine* sm = new StateMachine("walking");

  State* state0 = new State(mAtlasRobot, "0");
  State* state1 = new State(mAtlasRobot, "1");
  State* state2 = new State(mAtlasRobot, "2");
  State* state3 = new State(mAtlasRobot, "3");

  TerminalCondition* cond0 = new TimerCondition(state0, 0.3);
  TerminalCondition* cond1 = new BodyContactCondition(state1, _getRightFoot());
  TerminalCondition* cond2 = new TimerCondition(state2, 0.3);
  TerminalCondition* cond3 = new BodyContactCondition(state3, _getLeftFoot());

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);
  state2->setTerminalCondition(cond2);
  state3->setTerminalCondition(cond3);

  state0->setNextState(state1);
  state1->setNextState(state2);
  state2->setNextState(state3);
  state3->setNextState(state0);

  // Set stance foot
  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToLeftFoot();
  state2->setStanceFootToRightFoot();
  state3->setStanceFootToRightFoot();

  // Set global desired pelvis angle
  state0->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state1->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state2->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state3->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state0->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);
  state1->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);
  state2->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);
  state3->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);

  // Set desired joint position
  //-- State 0
  //---- pelvis
  state0->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state0->setDesiredJointPosition(14, -swh02); // right hip
  state0->setDesiredJointPosition(18, -swk02); // right knee
  state0->setDesiredJointPosition(22, -swa02); // right ankle
  //---- stance leg
  state0->setDesiredJointPosition(17, -stk02); // left knee
  state0->setDesiredJointPosition(21, -sta02); // left ankle
  //---- arm
  state0->setDesiredJointPosition(15, DART_RADIAN * -20.00); // left arm
  state0->setDesiredJointPosition(16, DART_RADIAN * +10.00); // right arm
  state0->setDesiredJointPosition(19, DART_RADIAN * -80.00); // left arm
  state0->setDesiredJointPosition(20, DART_RADIAN * +80.00); // right arm
  //---- feedback gain for hip joints
  state0->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state0->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state0->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state0->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state0->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state0->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state0->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state0->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 1
  //---- pelvis
  state1->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state1->setDesiredJointPosition(13, -swh13); // left hip
  state1->setDesiredJointPosition(17, -swk13); // left knee
  state1->setDesiredJointPosition(21, -swa13); // left ankle
  //---- stance leg
  state1->setDesiredJointPosition(18, -stk13); // right knee
  state1->setDesiredJointPosition(22, -sta13); // right ankle
  //---- arm
  state1->setDesiredJointPosition(15, DART_RADIAN * +10.00); // left arm
  state1->setDesiredJointPosition(16, DART_RADIAN * -20.00); // right arm
  state1->setDesiredJointPosition(19, DART_RADIAN * -80.00); // left arm
  state1->setDesiredJointPosition(20, DART_RADIAN * +80.00); // right arm
  //---- feedback gain for hip joints
  state1->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state1->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state1->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state1->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state1->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state1->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state1->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state1->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 2
  //---- pelvis
  state2->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state2->setDesiredJointPosition(13, -swh02); // left hip
  state2->setDesiredJointPosition(17, -swk02); // left knee
  state2->setDesiredJointPosition(21, -swa02); // left ankle
  //---- stance leg
  state2->setDesiredJointPosition(18, -stk02); // right knee
  state2->setDesiredJointPosition(22, -sta02); // right ankle
  //---- arm
  state2->setDesiredJointPosition(15, DART_RADIAN * +10.00); // left arm
  state2->setDesiredJointPosition(16, DART_RADIAN * -20.00); // right arm
  state2->setDesiredJointPosition(19, DART_RADIAN * -80.00); // left arm
  state2->setDesiredJointPosition(20, DART_RADIAN * +80.00); // right arm
  //---- feedback gain for hip joints
  state2->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state2->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state2->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state2->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state2->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state2->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state2->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state2->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 3
  //---- pelvis
  state3->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state3->setDesiredJointPosition(14, -swh13); // right hip
  state3->setDesiredJointPosition(18, -swk13); // right knee
  state3->setDesiredJointPosition(22, -swa13); // right ankle
  //---- stance leg
  state3->setDesiredJointPosition(17, -stk13); // left knee
  state3->setDesiredJointPosition(21, -sta13); // left ankle
  //---- arm
  state3->setDesiredJointPosition(15, DART_RADIAN * -20.00); // left arm
  state3->setDesiredJointPosition(16, DART_RADIAN * +10.00); // right arm
  state3->setDesiredJointPosition(19, DART_RADIAN * -80.00); // left arm
  state3->setDesiredJointPosition(20, DART_RADIAN * +80.00); // right arm
  //---- feedback gain for hip joints
  state3->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state3->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state3->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state3->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state3->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state3->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state3->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state3->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

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
  const double cd = 0.5;
  const double cv = 0.2;

  const double pelvis   = DART_RADIAN * -10.0;  // angle b/w pelvis and torso

  const double swh01 =  0.50;  // swing hip
  const double swk01 = -1.10;  // swing knee
  const double swa01 =  0.60;  // swing angle
  const double stk01 = -0.05;  // stance knee
  const double sta01 =  0.00;  // stance ankle

  StateMachine* sm = new StateMachine("running");

  State* state0 = new State(mAtlasRobot, "0");
  State* state1 = new State(mAtlasRobot, "1");

  TerminalCondition* cond0 = new TimerCondition(state0, 0.15);
  TerminalCondition* cond1 = new TimerCondition(state1, 0.15);

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);

  state0->setNextState(state1);
  state1->setNextState(state0);

  // Set stance foot
  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToRightFoot();

  // Set global desired pelvis angle
  state0->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state1->setDesiredPelvisGlobalAngleOnSagital(DART_RADIAN * 0.0);
  state0->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);
  state1->setDesiredPelvisGlobalAngleOnCoronal(DART_RADIAN * 0.0);

  // Set desired joint position
  //-- State 0
  //---- pelvis
  state0->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state0->setDesiredJointPosition(14, -swh01); // right hip
  state0->setDesiredJointPosition(18, -swk01); // right knee
  state0->setDesiredJointPosition(22, -swa01); // right ankle
  //---- stance leg
  state0->setDesiredJointPosition(17, -stk01); // left knee
  state0->setDesiredJointPosition(21, -sta01); // left ankle
  //---- arm
  state0->setDesiredJointPosition(15, DART_RADIAN * -45.00); // left arm
  state0->setDesiredJointPosition(16, DART_RADIAN * +15.00); // right arm
  state0->setDesiredJointPosition(19, DART_RADIAN * -80.00); // left arm
  state0->setDesiredJointPosition(20, DART_RADIAN * +80.00); // right arm
//  state0->setDesiredJointPosition(23, DART_RADIAN * +90.00); // left arm
//  state0->setDesiredJointPosition(24, DART_RADIAN * +90.00); // right arm
//  state0->setDesiredJointPosition(27, DART_RADIAN * +90.00); // left arm
//  state0->setDesiredJointPosition(28, DART_RADIAN * -90.00); // right arm
  //---- feedback gain for hip joints
  state0->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state0->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state0->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state0->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state0->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state0->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state0->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state0->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  //-- State 2
  //---- pelvis
  state1->setDesiredJointPosition( 9, -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state1->setDesiredJointPosition(13, -swh01); // left hip
  state1->setDesiredJointPosition(17, -swk01); // left knee
  state1->setDesiredJointPosition(21, -swa01); // left ankle
  //---- stance leg
  state1->setDesiredJointPosition(18, -stk01); // right knee
  state1->setDesiredJointPosition(22, -sta01); // right ankle
  //---- arm
  state1->setDesiredJointPosition(15, DART_RADIAN * +15.00); // left arm
  state1->setDesiredJointPosition(16, DART_RADIAN * -45.00); // right arm
  state1->setDesiredJointPosition(19, DART_RADIAN * -80.00); // left arm
  state1->setDesiredJointPosition(20, DART_RADIAN * +80.00); // right arm
//  state1->setDesiredJointPosition(23, DART_RADIAN * +90.00); // left arm
//  state1->setDesiredJointPosition(24, DART_RADIAN * +90.00); // right arm
//  state1->setDesiredJointPosition(27, DART_RADIAN * +90.00); // left arm
//  state1->setDesiredJointPosition(28, DART_RADIAN * -90.00); // right arm
  //---- feedback gain for hip joints
  state1->setFeedbackCoronalCOMDistance(10, -cd);  // coronal left hip
  state1->setFeedbackCoronalCOMVelocity(10, -cv);  // coronal left hip
  state1->setFeedbackCoronalCOMDistance(11, -cd);  // coronal right hip
  state1->setFeedbackCoronalCOMVelocity(11, -cv);  // coronal right hip
  state1->setFeedbackSagitalCOMDistance(13, -cd);  // sagital left hip
  state1->setFeedbackSagitalCOMVelocity(13, -cv);  // sagital left hip
  state1->setFeedbackSagitalCOMDistance(14, -cd);  // sagital right hip
  state1->setFeedbackSagitalCOMVelocity(14, -cv);  // sagital right hip

  sm->addState(state0);
  sm->addState(state1);

  sm->setInitialState(state0);

  return sm;
}

//==============================================================================
void Controller::_setJointDamping()
{
  for (size_t i = 1; i < mAtlasRobot->getNumBodyNodes(); ++i)
  {
    Joint* joint = mAtlasRobot->getJoint(i);
    if (joint->getDof() > 0)
    {
      for (size_t j = 0; j < joint->getDof(); ++j)
        joint->setDampingCoefficient(j, 80.0);
    }
  }
}

//==============================================================================
BodyNode* Controller::_getLeftFoot() const
{
  return mAtlasRobot->getBodyNode("l_foot");
}

//==============================================================================
BodyNode* Controller::_getRightFoot() const
{
  return mAtlasRobot->getBodyNode("r_foot");
}

//==============================================================================
bool Controller::_containStateMachine(StateMachine* _stateMachine)
{
  for (vector<StateMachine*>::iterator it = mStateMachines.begin();
       it != mStateMachines.end(); ++it)
  {
    if (*it == _stateMachine)
      return true;
  }

  return false;
}

//==============================================================================
bool Controller::_containStateMachine(const string& _name)
{
  return _containStateMachine(_findStateMachine(_name));
}

//==============================================================================
StateMachine* Controller::_findStateMachine(const string& _name)
{
  StateMachine* stateMachine = NULL;

  for (vector<StateMachine*>::iterator it = mStateMachines.begin();
       it != mStateMachines.end(); ++it)
  {
    if ((*it)->getName() == _name)
    {
      stateMachine = *it;
      break;
    }
  }

  return stateMachine;
}

