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

#include "apps/atlasRobot/StateMachine.h"

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Shape.h"
//#include "dart/constraint/OldConstraintDynamics.h"
#include "dart/collision/CollisionDetector.h"

#include "apps/atlasRobot/State.h"

// Macro for functions not implemented yet
#define NOT_YET(FUNCTION) std::cout << #FUNCTION\
                                  << "Not implemented yet."\
                                  << std::endl;

using namespace std;

using namespace dart::dynamics;
using namespace dart::constraint;

//==============================================================================
StateMachine::StateMachine(const std::string& _name)
  : mName(_name),
    mCurrentState(NULL),
    mBeginTime(0.0),
    mEndTime(0.0),
    mFrame(0),
    mElapsedTime(0.0)
{

}

//==============================================================================
StateMachine::~StateMachine()
{
  for (vector<State*>::iterator it = mStates.begin();
       it != mStates.end(); ++it)
  {
    delete *it;
  }
}

//==============================================================================
void StateMachine::setName(const std::string& _name)
{
  mName = _name;
}

//==============================================================================
const std::string& StateMachine::getName() const
{
  return mName;
}

//==============================================================================
void StateMachine::addState(State* _state)
{
  assert(_state != NULL && "Invalid state");
  assert(!_containState(_state) && "_state shouldn't be in mStates");

  mStates.push_back(_state);
}

//==============================================================================
void StateMachine::setInitialState(State* _state)
{
  assert(_state != NULL);
  assert(_containState(_state));

  mCurrentState = _state;
}

//==============================================================================
void StateMachine::begin(double _currentTime)
{
//  dtmsg << "StateMachine [" << getName() << "]: begin()." << endl;

  mBeginTime = _currentTime;
  mFrame = 0;
  mElapsedTime = 0.0;
}

//==============================================================================
void StateMachine::computeControlForce(double _dt)
{
  assert(mCurrentState != NULL && "Invaild current state.");

  // Check transition is needed from current state
  if (mCurrentState->isTerminalConditionSatisfied())
    transiteTo(mCurrentState->getNextState(), mBeginTime + mElapsedTime);

  // Update control force
  mCurrentState->computeControlForce(_dt);

  mElapsedTime += _dt;
  mFrame++;
}

//==============================================================================
void StateMachine::end(double _currentTime)
{
  mEndTime = _currentTime;

//  dtmsg << "StateMachine [" << getName() << "]: end()." << endl;
}

//==============================================================================
State* StateMachine::getCurrentState()
{
  return mCurrentState;
}

//==============================================================================
void StateMachine::transiteToNextState(double _currentTime)
{
  transiteTo(mCurrentState->getNextState(), _currentTime);
}

//==============================================================================
void StateMachine::transiteTo(State* _state, double _currentTime)
{
  assert(_containState(_state) && "_state should be in mStates");

  string prevStateName = mCurrentState->getName();
  string nextStateName = _state->getName();

  // Finish current state
  mCurrentState->end(_currentTime);

  // Transite to _state
  mCurrentState = _state;
  mCurrentState->begin(_currentTime);

  dtmsg << "Transition: ["
        << prevStateName << "] --> ["
        << nextStateName << "]." << endl;
}

//==============================================================================
void StateMachine::transiteTo(string& _stateName, double _currentTime)
{
  // _state should be in mStates
  State* state = _findState(_stateName);

  assert(state != NULL && "Invaild state.");

  transiteTo(state, _currentTime);
}

//==============================================================================
void StateMachine::transiteTo(size_t _idx, double _currentTime)
{
  assert(0 <= _idx && _idx <= mStates.size() && "Invalid index of State.");

  transiteTo(mStates[_idx], _currentTime);
}

//==============================================================================
bool StateMachine::_containState(State* _state)
{
  for (vector<State*>::iterator it = mStates.begin();
       it != mStates.end(); ++it)
  {
    if (*it == _state)
      return true;
  }

  return false;
}

//==============================================================================
bool StateMachine::_containState(const string& _name)
{
  return _containState(_findState(_name));
}

//==============================================================================
State* StateMachine::_findState(const string& _name)
{
  State* state = NULL;

  for (vector<State*>::iterator it = mStates.begin();
       it != mStates.end(); ++it)
  {
    if ((*it)->getName() == _name)
    {
      state = *it;
      break;
    }
  }

  return state;
}





