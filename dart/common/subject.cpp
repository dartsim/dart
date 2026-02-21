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

#include "dart/common/subject.hpp"

#include "dart/common/observer.hpp"

namespace dart {
namespace common {

//==============================================================================
Subject::~Subject()
{
  sendDestructionNotification();
}

//==============================================================================
void Subject::sendDestructionNotification() const
{
  // Snapshot the observer list so that mutations during callbacks (peer
  // destruction, re-subscription) cannot cause iterator invalidation or
  // infinite loops.  mObservers keeps tracking live registrations so that
  // removeObserver() from a peer's destructor still works correctly.
  //
  // Note: observers that re-subscribe during their callback will be
  // registered again in mObservers after this function returns.  This is
  // harmless when called from ~Subject() because the object destructs
  // immediately after.
  auto snapshot = mObservers;
  for (Observer* observer : snapshot) {
    // Skip observers that were already removed by an earlier callback
    // (e.g. a peer's destructor called removeObserver on this subject).
    if (!mObservers.contains(observer)) {
      continue;
    }
    mObservers.erase(observer);
    observer->receiveDestructionNotification(this);
  }
}

//==============================================================================
void Subject::addObserver(Observer* _observer) const
{
  if (nullptr == _observer) {
    return;
  }

  if (mObservers.contains(_observer)) {
    return;
  }

  mObservers.insert(_observer);
  _observer->addSubject(this);
}

//==============================================================================
void Subject::removeObserver(Observer* _observer) const
{
  if (nullptr == _observer) {
    return;
  }

  if (!mObservers.contains(_observer)) {
    return;
  }

  mObservers.erase(_observer);
  _observer->removeSubject(this);
}

} // namespace common
} // namespace dart
