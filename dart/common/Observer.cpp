/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/common/Subject.hpp"
#include "dart/common/Observer.hpp"

namespace dart {
namespace common {

//==============================================================================
Observer::~Observer()
{
  std::set<const Subject*>::iterator it = mSubjects.begin(),
                                          end = mSubjects.end();
  while( it != end )
    (*(it++))->removeObserver(this);
  // We do this tricky iterator method to deal with the fact that mObservers
  // will be changing as we go through the loop
}

//==============================================================================
void Observer::receiveDestructionNotification(const Subject* _subject)
{
  removeSubject(_subject);
  handleDestructionNotification(_subject);
}

//==============================================================================
void Observer::handleDestructionNotification(const Subject*)
{
  // Do nothing
}

//==============================================================================
void Observer::addSubject(const Subject* _subject)
{
  if(nullptr == _subject)
    return;

  if(mSubjects.find(_subject) != mSubjects.end())
    return;

  mSubjects.insert(_subject);
  _subject->addObserver(this);
}

//==============================================================================
void Observer::removeSubject(const Subject* _subject)
{
  if(nullptr == _subject)
    return;

  if(mSubjects.find(_subject) == mSubjects.end())
    return;

  mSubjects.erase(_subject);
  _subject->removeObserver(this);
}

//==============================================================================
void Observer::removeAllSubjects()
{
  std::set<const Subject*>::iterator it = mSubjects.begin(),
                                          end = mSubjects.end();

  while(it != end)
    removeSubject(*(it++));
}

} // namespace common
} // namespace dart
