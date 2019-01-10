/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "StateMachine.hpp"

#include "State.hpp"

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
    mCurrentState(nullptr),
    mBeginTime(0.0),
    mEndTime(0.0),
    mFrame(0),
    mElapsedTime(0.0),
    mVerbosity(false)
{
  // Do nothing
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
  assert(_state != nullptr && "Invalid state");
  assert(!_containState(_state) && "_state shouldn't be in mStates");

  mStates.push_back(_state);
}

//==============================================================================
void StateMachine::setInitialState(State* _state)
{
  assert(_state != nullptr);
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
  assert(mCurrentState != nullptr && "Invaild current state.");

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

  if (mVerbosity)
  {
    dtmsg << "Transition: ["
          << prevStateName << "] --> ["
          << nextStateName << "]." << endl;
  }
}

//==============================================================================
void StateMachine::transiteTo(string& _stateName, double _currentTime)
{
  // _state should be in mStates
  State* state = _findState(_stateName);

  assert(state != nullptr && "Invaild state.");

  transiteTo(state, _currentTime);
}

//==============================================================================
void StateMachine::transiteTo(std::size_t _idx, double _currentTime)
{
  assert(_idx <= mStates.size() && "Invalid index of State.");

  transiteTo(mStates[_idx], _currentTime);
}

//==============================================================================
void StateMachine::setVerbosity(bool verbosity)
{
  mVerbosity = verbosity;
}

//==============================================================================
bool StateMachine::_containState(const State* _state) const
{
  for (vector<State*>::const_iterator it = mStates.begin();
       it != mStates.end(); ++it)
  {
    if (*it == _state)
      return true;
  }

  return false;
}

//==============================================================================
bool StateMachine::_containState(const string& _name) const
{
  return _containState(_findState(_name));
}

//==============================================================================
State* StateMachine::_findState(const string& _name) const
{
  State* state = nullptr;

  for (vector<State*>::const_iterator it = mStates.begin();
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
