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

#include "StateMachine.hpp"

#include "State.hpp"
#include "dart/common/Macros.hpp"

namespace dart_demos {
namespace atlas_simbicon {

//==============================================================================
StateMachine::StateMachine(const std::string& name)
  : mName(name),
    mCurrentState(nullptr),
    mBeginTime(0.0),
    mEndTime(0.0),
    mFrame(0),
    mElapsedTime(0.0),
    mVerbosity(false)
{
}

//==============================================================================
StateMachine::~StateMachine()
{
  for (auto* state : mStates)
    delete state;
}

//==============================================================================
void StateMachine::setName(const std::string& name)
{
  mName = name;
}

//==============================================================================
const std::string& StateMachine::getName() const
{
  return mName;
}

//==============================================================================
void StateMachine::addState(State* state)
{
  DART_ASSERT(state != nullptr && "Invalid state");
  DART_ASSERT(!_containState(state) && "state shouldn't be in mStates");

  mStates.push_back(state);
}

//==============================================================================
void StateMachine::setInitialState(State* state)
{
  DART_ASSERT(state != nullptr);
  DART_ASSERT(_containState(state));

  mCurrentState = state;
}

//==============================================================================
void StateMachine::begin(double currentTime)
{
  mBeginTime = currentTime;
  mFrame = 0;
  mElapsedTime = 0.0;
}

//==============================================================================
void StateMachine::computeControlForce(double dt)
{
  DART_ASSERT(mCurrentState != nullptr && "Invalid current state.");

  if (mCurrentState->isTerminalConditionSatisfied())
    transiteTo(mCurrentState->getNextState(), mBeginTime + mElapsedTime);

  mCurrentState->computeControlForce(dt);

  mElapsedTime += dt;
  mFrame++;
}

//==============================================================================
void StateMachine::end(double currentTime)
{
  mEndTime = currentTime;
}

//==============================================================================
State* StateMachine::getCurrentState()
{
  return mCurrentState;
}

//==============================================================================
void StateMachine::transiteToNextState(double currentTime)
{
  transiteTo(mCurrentState->getNextState(), currentTime);
}

//==============================================================================
void StateMachine::transiteTo(State* state, double currentTime)
{
  DART_ASSERT(_containState(state) && "state should be in mStates");

  const std::string prevStateName = mCurrentState->getName();
  const std::string nextStateName = state->getName();

  mCurrentState->end(currentTime);
  mCurrentState = state;
  mCurrentState->begin(currentTime);

  if (mVerbosity) {
    dtmsg << "Transition: [" << prevStateName << "] --> [" << nextStateName
          << "]." << std::endl;
  }
}

//==============================================================================
void StateMachine::transiteTo(std::string& stateName, double currentTime)
{
  State* state = _findState(stateName);
  DART_ASSERT(state != nullptr && "Invalid state.");
  transiteTo(state, currentTime);
}

//==============================================================================
void StateMachine::transiteTo(std::size_t idx, double currentTime)
{
  DART_ASSERT(idx <= mStates.size() && "Invalid index of State.");
  transiteTo(mStates[idx], currentTime);
}

//==============================================================================
void StateMachine::setVerbosity(bool verbosity)
{
  mVerbosity = verbosity;
}

//==============================================================================
bool StateMachine::_containState(const State* state) const
{
  for (const auto* s : mStates) {
    if (s == state)
      return true;
  }
  return false;
}

//==============================================================================
bool StateMachine::_containState(const std::string& name) const
{
  return _containState(_findState(name));
}

//==============================================================================
State* StateMachine::_findState(const std::string& name) const
{
  for (auto* state : mStates) {
    if (state->getName() == name)
      return state;
  }
  return nullptr;
}

} // namespace atlas_simbicon
} // namespace dart_demos
