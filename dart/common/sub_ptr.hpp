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

#ifndef DART_COMMON_SUB_PTR_HPP_
#define DART_COMMON_SUB_PTR_HPP_

#include "dart/common/Observer.hpp"

namespace dart {
namespace common {

/// sub_ptr is a pointer to a Subject. It can be used as a pointer to any class
/// that publicly inherits Subject. If the instance that it is pointing to is
/// ever destroyed, the sub_ptr class will start pointing to a nullptr. You can
/// check the return of sub_ptr::valid() to see if the pointer is still valid.
template <class T>
class sub_ptr : public Observer
{
public:
  /// Default constructor
  sub_ptr();

  /// Alternative constructor. _ptr must be a valid pointer when passed to this
  /// constructor.
  sub_ptr(T* _ptr);

  /// Change the Subject of this sub_ptr
  sub_ptr& operator = (const sub_ptr& _sp);

  /// Change the Subject of this sub_ptr
  sub_ptr& operator = (T* _ptr);

  /// Implicit conversion to pointer type
  operator T*() const;

  /// Dereferencing operator
  T& operator*() const;

  /// Dereferencing operation
  T* operator->() const;

  /// Get the Subject of this sub_ptr
  T* get() const;

  /// Set the subject of this sub_ptr
  void set(T* _ptr);

  /// True if and only if this sub_ptr still points to a valid Subject
  bool valid();

protected:

  void handleDestructionNotification(const Subject* _subject) override;

  /// Store the pointer to the full object
  T* mT;

  /// Store the pointer to the virtual Subject base
  Subject* mSubjectBase;
};

} // namespace common

// Make an alias for sub_ptr in the dart namespace for convenience
template <class T>
using sub_ptr = common::sub_ptr<T>;

} // namespace dart

#include "dart/common/detail/sub_ptr.hpp"

#endif // DART_COMMON_SUB_PTR_HPP_
