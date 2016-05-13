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

#ifndef DART_COMMON_SUBJECT_HPP_
#define DART_COMMON_SUBJECT_HPP_

#include <set>

namespace dart {
namespace common {

class Observer;

/// The Subject class is a base class for any object that wants to report when
/// it gets destroyed. This is useful for complex frameworks in which it is
/// difficult or impossible to know when an object or resource might destroyed
/// by some other part of the code, or when special cleanup might be needed
/// upon the destruction of an object. Simply by inheriting the Subject class,
/// any class can have the destruction notification feature.
///
/// Note that the Subject class should ALWAYS be virtually inherited. No other
/// special considerations are needed when virtually inheriting the Subject
/// class.
///
/// dart::sub_ptr is a templated smart pointer that will change itself into a
/// nullptr when its Subject is destroyed. It offers one of the easiest ways to
/// take advantage of the Subject/Observer pattern.
class Subject
{
public:

  friend class Observer;

  /// Destructor will notify all Observers that it is destructing
  virtual ~Subject();

protected:

  /// Send a destruction notification to all Observers. This will cause all
  /// Observers to behave as if this Subject has been permanently deleted, so it
  /// should only be called when that behavior is desired.
  void sendDestructionNotification() const;

  /// Add an Observer to the list of Observers
  void addObserver(Observer* _observer) const;

  /// Remove an Observer from the list of Observers
  void removeObserver(Observer* _observer) const;

  /// List of current Observers
  mutable std::set<Observer*> mObservers;

};

} // namespace common
} // namespace dart

#endif // DART_COMMON_SUBJECT_HPP_
