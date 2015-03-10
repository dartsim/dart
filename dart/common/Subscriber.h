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

#ifndef DART_COMMON_SUBSCRIBER_H_
#define DART_COMMON_SUBSCRIBER_H_

#include <set>

namespace dart {
namespace common {

class Publisher;

class Subscriber
{
public:

  friend class Publisher;

  /// Destructor will notify all Subscriptions that it is destructing
  virtual ~Subscriber();

protected:

  /// Called whenever a Subscription sends out a notice. Override this in order
  /// to respond to notifications
  virtual void receiveNotification(const Publisher* _subscription,
                                   int _notice);

  /// Called whenever a Subscription is destroyed (or sends out a destruction
  /// notification). Override handleDestructionNotification() in order to
  /// customize your class's response to destruction notifications.
  void receiveDestructionNotification(const Publisher* _subscription);

  /// Called by receiveDestructionNotification(). Override this function to
  /// customize your class's response to destruction notifications.
  virtual void handleDestructionNotification(const Publisher* _subscription);

  /// Add a Subscription for this Subscriber
  void addSubscription(const Publisher* _subscription);

  /// Remove a Subscription from this Subscriber
  void removeSubscription(const Publisher* _subscription);

  /// Remove all Subscriptions from this Subscriber
  void clearSubscriptions();

  /// List of current Subscriptions for this Subscriber
  std::set<const Publisher*> mSubscriptions;

};

} // namespace dart
} // namespace common

#endif // DART_COMMON_SUBSCRIBER_H_
