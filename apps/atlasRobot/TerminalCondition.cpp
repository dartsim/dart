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

#include "apps/atlasRobot/TerminalCondition.h"

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/PointMass.h"
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
TerminalCondition::TerminalCondition(State* _state)
  : mState(_state)
{
  assert(_state != NULL);
}

//==============================================================================
TerminalCondition::~TerminalCondition()
{

}

//==============================================================================
TimerCondition::TimerCondition(State* _state, double _duration)
  : TerminalCondition(_state),
    mDuration(_duration)
{

}

//==============================================================================
TimerCondition::~TimerCondition()
{

}

//==============================================================================
bool TimerCondition::isSatisfied()
{
  if (mState->getElapsedTime() > mDuration)
    return true;
  else
    return false;
}

//==============================================================================
BodyContactCondition::BodyContactCondition(State* _state, BodyNode* _body)
  : TerminalCondition(_state),
    mBodyNode(_body)
{
  assert(_state != NULL);
  assert(_body != NULL);
}

//==============================================================================
BodyContactCondition::~BodyContactCondition()
{
}

//==============================================================================
bool BodyContactCondition::isSatisfied()
{
  SoftBodyNode* soft = dynamic_cast<SoftBodyNode*>(mBodyNode);
  if (soft)
  {
    for (size_t i = 0; i < soft->getNumPointMasses(); ++i)
    {
      PointMass* pm = soft->getPointMass(i);
      if (pm->isColliding() > 0)
        return true;
    }
  }

  // TODO(JS): Need more elegant condition check method
  if (mBodyNode->isColliding() > 0)
  {
//    dtmsg << "BodyNode [" << mBodyNode->getName() << "] is in contact."
//          << std::endl;
    return true;
  }
  else
  {
//    dtmsg << "Waiting for BodyNode [" << mBodyNode->getName()
//          << "] is in contact."
//          << std::endl;
    return false;
  }
}

