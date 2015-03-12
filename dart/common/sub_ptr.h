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

#ifndef DART_COMMON_SUB_PTR_H_
#define DART_COMMON_SUB_PTR_H_

#include "dart/common/Subscriber.h"

namespace dart {
namespace common {

/// sub_ptr is a subscribed pointer. It can be used as a pointer to any class
/// that publicly inherits Subscription. If the instance that it is pointing to
/// is ever destroyed, the sub_ptr class will start pointing to a nullptr.
/// You can check the return of sub_ptr::valid() to see if the pointer is still
/// valid. You can also check the latest notification sent out by the
/// subscription using sub_ptr::getLatestNotification()
template <class T>
class sub_ptr : public Subscriber
{
public:
  /// Default constructor
  sub_ptr() : mSubscription(nullptr), mLatestNotification(0) { }

  /// Alternative constructor. _ptr must be a valid pointer when passed to this
  /// constructor.
  sub_ptr(T* _ptr) : mSubscription(nullptr), mLatestNotification(0)
  {
    set(_ptr);
  }

  /// Change the subscription of this sub_ptr
  sub_ptr& operator = (const sub_ptr& _sp)
  {
    set(_sp.get());
    return *this;
  }

  /// Change the subscription of this sub_ptr
  sub_ptr& operator = (T* _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Implicit conversion to pointer type
  operator T* () const { return mSubscription; }

  /// Dereferencing operator
  T& operator*() const { return *mSubscription; }

  /// Dereferencing operation
  T* operator->() const { return mSubscription; }

  /// Get the subscription of this sub_ptr
  T* get() const { return mSubscription; }

  void set(T* _ptr)
  {
    if(mSubscription == _ptr)
      return;

    removeSubscription(mSubscription);
    mSubscription = _ptr;
    mLatestNotification = 0;
    addSubscription(mSubscription);
  }

  /// Get the latest notification produced by the subscription of this sub_ptr
  int getLatestNotification() const { return mLatestNotification; }

  /// True if and only if this sub_ptr still points to a valid subscription
  bool valid() { return mSubscription != nullptr; }

protected:
  /// Saves the latest notification received from its Subscription
  virtual void receiveNotification(const Publisher* _subscription,
                                   int _notice) override
  {
    if(_subscription == mSubscription)
      mLatestNotification = _notice;
  }

  virtual void handleDestructionNotification(
      const Publisher* _subscription) override
  {
    if(_subscription == mSubscription)
      mSubscription = nullptr;
  }

  /// Store the subscription pointer
  T* mSubscription;

  /// Store the latest notification
  int mLatestNotification;

};

} // namespace common

// Make an alias for sub_ptr in the dart namespace for convenience
template <class T>
using sub_ptr = common::sub_ptr<T>;

} // namespace dart

#endif // DART_COMMON_SUB_PTR_H_
