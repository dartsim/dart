/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#include <cassert>
#include <string>
#include <iostream>

#include "dart/common/Aspect.h"
#include "dart/common/Console.h"

namespace dart {
namespace common {

//==============================================================================
void Aspect::setAspectState(const State& /*otherState*/)
{
  // Do nothing
}

//==============================================================================
const Aspect::State* Aspect::getAspectState() const
{
  return nullptr;
}

//==============================================================================
void Aspect::setAspectProperties(const Properties& /*someProperties*/)
{
  // Do nothing
}

//==============================================================================
const Aspect::Properties* Aspect::getAspectProperties() const
{
  return nullptr;
}

//==============================================================================
Aspect::Aspect(Composite* manager)
{
  if(nullptr == manager)
  {
    dterr << "[Aspect::constructor] You are not allowed to construct an Aspect "
          << "outside of an Composite!\n";
    assert(false);
  }
}

//==============================================================================
void Aspect::setComposite(Composite* /*newComposite*/)
{
  // Do nothing
}

//==============================================================================
void Aspect::loseComposite(Composite* /*oldComposite*/)
{
  // Do nothing
}

} // namespace common
} // namespace dart
