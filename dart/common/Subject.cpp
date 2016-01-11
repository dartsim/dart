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

#include "dart/common/Subject.h"
#include "dart/common/Observer.h"

namespace kido {
namespace common {

//==============================================================================
Subject::~Subject()
{
  sendDestructionNotification();
}

//==============================================================================
void Subject::sendDestructionNotification() const
{
  std::set<Observer*>::iterator sub = mObservers.begin(),
                                  end = mObservers.end();
  while( sub != end )
    (*(sub++))->receiveDestructionNotification(this);
  // We do this tricky iterator method to deal with the fact that mObservers
  // will be changing as we go through the loop
}

//==============================================================================
void Subject::addObserver(Observer* _observer) const
{
  if(nullptr == _observer)
    return;

  if(mObservers.find(_observer) != mObservers.end())
    return;

  mObservers.insert(_observer);
  _observer->addSubject(this);
}

//==============================================================================
void Subject::removeObserver(Observer* _observer) const
{
  if(nullptr == _observer)
    return;

  if(mObservers.find(_observer) == mObservers.end())
    return;

  mObservers.erase(_observer);
  _observer->removeSubject(this);
}

} // namespace common
} // namespace kido
