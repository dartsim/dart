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

#ifndef DART_DYNAMICS_DETAIL_JOINTPTR_HPP_
#define DART_DYNAMICS_DETAIL_JOINTPTR_HPP_

#include "dart/dynamics/detail/BodyNodePtr.hpp"

namespace dart {
namespace dynamics {

/// TemplateJointPtr is a templated class that enables users to create a strong
/// reference-counting JointPtr. Holding onto a JointPtr will ensure that the
/// child BodyNode (and by extension, Skeleton) corresponding to a Joint does
/// not get deleted. If the child BodyNode of this Joint replaces its parent
/// Joint, then this smart pointer will reference the new parent Joint, because
/// the old one will have been deleted.
template <class JointT, class BodyNodeT>
class TemplateJointPtr
{
public:

  template<class, class> friend class TemplateJointPtr;

  typedef JointT element_type;

  /// Default constructor
  TemplateJointPtr() = default;

  /// Typical constructor. _ptr must be a valid pointer (or a nullptr) when
  /// passed to this constructor
  TemplateJointPtr(JointT* _ptr) { set(_ptr); }

  /// Constructor that takes in a strong JointPtr
  template <class OtherJointT, class OtherBodyNodeT>
  TemplateJointPtr(
      const TemplateJointPtr<OtherJointT, OtherBodyNodeT>& _jptr)
  {
    set(_jptr.get());
  }

  /// Assignment operator
  TemplateJointPtr& operator = (JointT* _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Assignment operator for JointPtrs
  template <class OtherJointT, class OtherBodyNodeT>
  TemplateJointPtr& operator = (
      const TemplateJointPtr<OtherJointT, OtherBodyNodeT>& _jptr)
  {
    set(_jptr.get());
    return *this;
  }

  /// Implicit conversion
  operator JointT*() const { return get(); }

  /// Dereferencing operator
  JointT& operator*() const { return *get(); }

  /// Dereferencing operation
  JointT* operator->() const { return get(); }

  /// Get the raw Joint pointer
  JointT* get() const
  {
    if(nullptr == mBodyNodePtr)
      return nullptr;

    return mBodyNodePtr->getParentJoint();
  }

  /// Get the BodyNode that this JointPtr is tied to
  TemplateBodyNodePtr<BodyNodeT> getBodyNodePtr() const
  {
    return mBodyNodePtr;
  }

  /// Set the Joint for this JointPtr
  void set(JointT* _ptr)
  {
    if(nullptr == _ptr)
    {
      mBodyNodePtr = nullptr;
      return;
    }

    mBodyNodePtr = _ptr->getChildBodyNode();
  }

  //----------------------------------------------------------------------------
  /// \{ \name Comparison operators
  //----------------------------------------------------------------------------

  /// Equality
  template <class OtherJointT, class OtherBodyNodeT>
  bool operator == (const TemplateJointPtr<OtherJointT,
                    OtherBodyNodeT>& _rhs) const
  {
    return mBodyNodePtr == _rhs.mBodyNodePtr;
  }

  /// Inequality
  template <class OtherJointT, class OtherBodyNodeT>
  bool operator != (const TemplateJointPtr<OtherJointT,
                    OtherBodyNodeT>& _rhs) const
  {
    return !( *this == _rhs );
  }

  /// Less than
  template <class OtherJointT, class OtherBodyNodeT>
  bool operator < (const TemplateJointPtr<OtherJointT,
                   OtherBodyNodeT>& _rhs) const
  {
    return (mBodyNodePtr < _rhs.mBodyNodePtr);
  }

  /// Greater than
  template <class OtherJointT, class OtherBodyNodeT>
  bool operator > (const TemplateJointPtr<OtherJointT,
                   OtherBodyNodeT>& _rhs) const
  {
    return (mBodyNodePtr > _rhs.mBodyNodePtr);
  }

  /// Less than or equal to
  template <class OtherJointT, class OtherBodyNodeT>
  bool operator <= (const TemplateJointPtr<OtherJointT,
                    OtherBodyNodeT>& _rhs) const
  {
    return (*this < _rhs) || (*this == _rhs);
  }

  /// Greater than or equal to
  template <class OtherJointT, class OtherBodyNodeT>
  bool operator >= (const TemplateJointPtr<OtherJointT,
                    OtherBodyNodeT>& _rhs) const
  {
    return (*this > _rhs) || (*this == _rhs);
  }

  /// \}

private:
  /// Reference-holding pointer to the child BodyNode of this Joint
  TemplateBodyNodePtr<BodyNodeT> mBodyNodePtr;
};

/// TemplateWeakJointPtr is a templated class that enables users to create a
/// non-reference-holding WeakJointPtr. Holding onto a WeakJointPtr will NOT
/// prevent anything from getting deleted, but you can use lock() to check
/// whether the Joint still exists. If it does exist, it will return a valid
/// JointPtr. Otherwise it will return a nullptr JointPtr.
template <class JointT, class BodyNodeT>
class TemplateWeakJointPtr
{
public:

  template<class, class> friend class TemplateWeakJointPtr;

  /// Default constructor
  TemplateWeakJointPtr() = default;

  /// Typical constructor. _ptr must be a valid pointer (or a nullptr) when
  /// passed to this constructor
  TemplateWeakJointPtr(JointT* _ptr) { set(_ptr); }

  /// Constructor that takes in a WeakJointPtr
  template <class OtherJointT, class OtherBodyNodeT>
  TemplateWeakJointPtr(
      const TemplateWeakJointPtr<OtherJointT, OtherBodyNodeT>& _weakPtr)
  {
    set(_weakPtr);
  }

  /// Constructor that takes in a strong JointPtr
  template <class OtherJointT, class OtherBodyNodeT>
  TemplateWeakJointPtr(
      const TemplateJointPtr<OtherJointT, OtherBodyNodeT>& _strongPtr)
  {
    set(_strongPtr.get());
  }

  /// Assignment operator for raw Joint pointers
  TemplateWeakJointPtr& operator = (JointT* _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Assignment operator for WeakJointPtrs
  template <class OtherJointT, class OtherBodyNodeT>
  TemplateWeakJointPtr& operator = (
      const TemplateWeakJointPtr<OtherJointT, OtherBodyNodeT>& _weakPtr)
  {
    set(_weakPtr);
    return *this;
  }

  /// Assignment operator for strong JointPtrs
  template <class OtherJointT, class OtherBodyNodeT>
  TemplateWeakJointPtr& operator = (
      const TemplateJointPtr<OtherJointT, OtherBodyNodeT>& _strongPtr)
  {
    set(_strongPtr.get());
    return *this;
  }

  /// Locks the Joint reference to ensure that the referenced Joint is currently
  /// still available. If the Joint is not available any longer (i.e. has been
  /// deleted), then this will return a nullptr.
  TemplateJointPtr<JointT, BodyNodeT> lock() const
  {
    TemplateBodyNodePtr<BodyNodeT> bodyNode = mWeakBodyNode.lock();
    if(nullptr == bodyNode)
      return nullptr;

    return TemplateJointPtr<JointT, BodyNodeT>(bodyNode->getParentJoint());
  }

  /// Set the Joint for this WeakJointPtr
  void set(JointT* _ptr)
  {
    if(nullptr == _ptr)
    {
      mWeakBodyNode = nullptr;
      return;
    }

    mWeakBodyNode = _ptr->getChildBodyNode();
  }

  /// Set the Joint for this WeakJointPtr based on another WeakJointPtr
  template <class OtherJointT, class OtherBodyNodeT>
  void set(const TemplateWeakJointPtr<OtherJointT, OtherBodyNodeT>& _weakPtr)
  {
    mWeakBodyNode = _weakPtr.mWeakBodyNode;
  }

private:
  /// Weak poiner to the child BodyNode of this Joint
  TemplateWeakBodyNodePtr<BodyNodeT> mWeakBodyNode;
};

} // namespace dart
} // namespace dynamics

#endif // DART_DYNAMICS_DETAIL_JOINTPTR_HPP_
