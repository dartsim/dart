/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_COMMON_OBSERVER_HPP_
#define DART_COMMON_OBSERVER_HPP_

#include <set>

namespace dart {
namespace common {

class Subject;

/// The Observer class should be inherited by any class that wants to respond
/// in a customized way to the destruction of a Subject. Simply override the
/// Observer::handleDestructionNotification() function to customize how your
/// class responds to the destruction of a Subject.
///
/// dart::sub_ptr is a templated smart pointer that will change itself into a
/// nullptr when its Subject is destroyed. It offers one of the easiest ways to
/// take advantage of the Subject/Observer pattern.
class Observer
{
public:

  friend class Subject;

  /// Destructor will notify all Subjects that it is destructing
  virtual ~Observer();

protected:

  /// Called whenever a Subject is destroyed (or sends out a destruction
  /// notification). Override handleDestructionNotification() in order to
  /// customize your class's response to destruction notifications.
  void receiveDestructionNotification(const Subject* _subject);

  /// Called by receiveDestructionNotification(). Override this function to
  /// customize your class's response to destruction notifications.
  virtual void handleDestructionNotification(const Subject* _subject);

  /// Add a Subject for this Observer
  void addSubject(const Subject* _subject);

  /// Remove a Subject from this Observer
  void removeSubject(const Subject* _subject);

  /// Remove all Subjects from this Observer
  void removeAllSubjects();

  /// List of current Subjects for this Observer
  std::set<const Subject*> mSubjects;

};

} // namespace dart
} // namespace common

#endif // DART_COMMON_OBSERVER_HPP_
