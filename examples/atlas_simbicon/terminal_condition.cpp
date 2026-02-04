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

#include "terminal_condition.hpp"

#include "dart/common/macros.hpp"
#include "state.hpp"

// Macro for functions not implemented yet
#define NOT_YET(FUNCTION)                                                      \
  std::cout << #FUNCTION << "Not implemented yet." << std::endl;

using namespace std;

using namespace dart::dynamics;
using namespace dart::constraint;

//==============================================================================
TerminalCondition::TerminalCondition(State* _state) : mState(_state)
{
  DART_ASSERT(_state != nullptr);
}

//==============================================================================
TerminalCondition::~TerminalCondition() {}

//==============================================================================
TimerCondition::TimerCondition(State* _state, double _duration)
  : TerminalCondition(_state), mDuration(_duration)
{
}

//==============================================================================
TimerCondition::~TimerCondition() {}

//==============================================================================
bool TimerCondition::isSatisfied()
{
  if (mState->getElapsedTime() > mDuration) {
    return true;
  } else {
    return false;
  }
}

//==============================================================================
BodyContactCondition::BodyContactCondition(
    State* _state,
    BodyNode* _body,
    dart::constraint::ConstraintSolver* constraintSolver)
  : TerminalCondition(_state),
    mBodyNode(_body),
    mConstraintSolver(constraintSolver)
{
  DART_ASSERT(_state != nullptr);
  DART_ASSERT(_body != nullptr);
}

//==============================================================================
BodyContactCondition::~BodyContactCondition() {}

//==============================================================================
bool BodyContactCondition::isSatisfied()
{
  SoftBodyNode* soft = dynamic_cast<SoftBodyNode*>(mBodyNode);
  if (soft) {
    for (std::size_t i = 0; i < soft->getNumPointMasses(); ++i) {
      PointMass* pm = soft->getPointMass(i);
      if (pm->isColliding()) {
        return true;
      }
    }
  }

  if (!mConstraintSolver) {
    return false;
  }

  const auto& result = mConstraintSolver->getLastCollisionResult();
  return result.inCollision(mBodyNode);
}
