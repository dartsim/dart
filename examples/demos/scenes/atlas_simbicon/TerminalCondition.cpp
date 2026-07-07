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

#include "TerminalCondition.hpp"

#include "State.hpp"
#include "dart/common/Macros.hpp"

namespace dart_demos {
namespace atlas_simbicon {

//==============================================================================
TerminalCondition::TerminalCondition(State* state) : mState(state)
{
  DART_ASSERT(state != nullptr);
}

//==============================================================================
TerminalCondition::~TerminalCondition() {}

//==============================================================================
TimerCondition::TimerCondition(State* state, double duration)
  : TerminalCondition(state), mDuration(duration)
{
}

//==============================================================================
TimerCondition::~TimerCondition() {}

//==============================================================================
bool TimerCondition::isSatisfied()
{
  return mState->getElapsedTime() > mDuration;
}

//==============================================================================
BodyContactCondition::BodyContactCondition(
    State* state, dart::dynamics::BodyNode* body)
  : TerminalCondition(state), mBodyNode(body)
{
  DART_ASSERT(state != nullptr);
  DART_ASSERT(body != nullptr);
}

//==============================================================================
BodyContactCondition::~BodyContactCondition() {}

//==============================================================================
bool BodyContactCondition::isSatisfied()
{
  auto* soft = dynamic_cast<dart::dynamics::SoftBodyNode*>(mBodyNode);
  if (soft) {
    for (std::size_t i = 0; i < soft->getNumPointMasses(); ++i) {
      if (soft->getPointMass(i)->isColliding())
        return true;
    }
  }

  DART_SUPPRESS_DEPRECATED_BEGIN
  const bool colliding = mBodyNode->isColliding();
  DART_SUPPRESS_DEPRECATED_END
  return colliding;
}

} // namespace atlas_simbicon
} // namespace dart_demos
