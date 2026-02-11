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
  // Drain from the beginning: erase each observer before invoking its callback
  // so that if the callback destroys a sibling observer (whose destructor calls
  // removeObserver), the sibling is safely erased from mObservers without
  // invalidating any iterator. A swap-into-local alternative is unsafe here
  // because it leaves dangling pointers in the local set when siblings are
  // destroyed. The iteration cap prevents infinite loops if a callback
  // re-registers an observer on this (dying) subject.
  const auto maxIterations = mObservers.size();
  for (std::size_t i = 0; i < maxIterations && !mObservers.empty(); ++i) {
    Observer* observer = *mObservers.begin();
    mObservers.erase(mObservers.begin());
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
