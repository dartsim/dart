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

#ifndef DART_DYNAMICS_DETAIL_DEGREEOFFREEDOMPTR_HPP_
#define DART_DYNAMICS_DETAIL_DEGREEOFFREEDOMPTR_HPP_

#include "dart/dynamics/detail/BodyNodePtr.hpp"
#include "dart/dynamics/InvalidIndex.hpp"

namespace dart {
namespace dynamics {

/// TemplateDegreeOfFreedomPtr is a templated class that enables users to create
/// a reference-counting DegreeOfFreedomPtr. Holding onto a DegreeOfFreedomPtr
/// will ensure that the BodyNode (and by extension, Skeleton) corresponding to
/// a DegreeOfFreedom does not get deleted. However, the DegreeOfFreedom itself
/// will be deleted if the parent Joint of the BodyNode is changed to a Joint
/// type that has a small number of DegreesOfFreedom than the local of the
/// DegreeOfFreedom that this DegreeOfFreedomPtr referred to. In such a case,
/// this will trigger and assertion in debug mode, or have a nullptr value if
/// not in debug mode.
template <class DegreeOfFreedomT, class BodyNodeT>
class TemplateDegreeOfFreedomPtr
{
public:

  template<class, class> friend class TemplateDegreeOfFreedomPtr;

  /// Default constructor
  TemplateDegreeOfFreedomPtr() = default;

  /// Typical constructor. _ptr must be a valid pointer (or a nullptr) when
  /// passed to this constructor
  TemplateDegreeOfFreedomPtr(DegreeOfFreedomT* _ptr) { set(_ptr); }

  /// Constructor that takes in a strong DegreeOfFreedomPtrs
  template <class OtherDegreeOfFreedomT, class OtherBodyNodeT>
  TemplateDegreeOfFreedomPtr(
      const TemplateDegreeOfFreedomPtr<OtherDegreeOfFreedomT,
      OtherBodyNodeT>& _dofp)
  {
    set(_dofp.get());
  }

  /// Assignment operator
  TemplateDegreeOfFreedomPtr& operator = (DegreeOfFreedomT* _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Assignment operator for DegreeOfFreedomPtrs
  template <class OtherDegreeOfFreedomT, class OtherBodyNodeT>
  TemplateDegreeOfFreedomPtr& operator = (
      const TemplateDegreeOfFreedomPtr<OtherDegreeOfFreedomT,
      OtherBodyNodeT>& _dofp)
  {
    set(_dofp.get());
    return *this;
  }

  /// Implicit conversion
  operator DegreeOfFreedomT*() const { return get(); }

  /// Dereferencing operator
  DegreeOfFreedomT& operator*() const { return *get(); }

  /// Dereferencing operation
  DegreeOfFreedomT* operator->() const { return get(); }

  /// Get the raw DegreeOfFreedom pointer
  DegreeOfFreedomT* get() const
  {
    if(nullptr == mBodyNodePtr)
      return nullptr;

    return mBodyNodePtr->getParentJoint()->getDof(mIndex);
  }

  /// Get the BodyNode that this DegreeOfFreedomPtr is tied to
  TemplateBodyNodePtr<BodyNodeT> getBodyNodePtr() const
  {
    return mBodyNodePtr;
  }

  /// Get the local generalized coordinate index that this DegreeOfFreedomPtr is
  /// tied to
  std::size_t getLocalIndex() const
  {
    if(nullptr == mBodyNodePtr)
      return INVALID_INDEX;

    return mIndex;
  }

  /// Set the DegreeOfFreedom for this DegreeOfFreedomPtr
  void set(DegreeOfFreedomT* _ptr)
  {
    if(nullptr == _ptr)
    {
      mBodyNodePtr = nullptr;
      return;
    }

    mBodyNodePtr = _ptr->getChildBodyNode();
    mIndex = _ptr->getIndexInJoint();
  }

  //----------------------------------------------------------------------------
  /// \{ \name Comparison operators
  //----------------------------------------------------------------------------

  /// Equality
  template <class OtherDofT, class OtherBodyNodeT>
  bool operator == (const TemplateDegreeOfFreedomPtr<OtherDofT,
                    OtherBodyNodeT>& _rhs)
  {
    if(nullptr == mBodyNodePtr && nullptr == _rhs.mBodyNodePtr)
      return true;

    if( (mBodyNodePtr == _rhs.mBodyNodePtr) && (mIndex == _rhs.mIndex) )
      return true;

    return false;
  }

  /// Inequality
  template <class OtherDofT, class OtherBodyNodeT>
  bool operator != (const TemplateDegreeOfFreedomPtr<OtherDofT,
                    OtherBodyNodeT>& _rhs)
  {
    return !( *this == _rhs );
  }

  /// Less than
  template <class OtherDofT, class OtherBodyNodeT>
  bool operator < (const TemplateDegreeOfFreedomPtr<OtherDofT,
                   OtherBodyNodeT>& _rhs)
  {
    if( mBodyNodePtr == _rhs.mBodyNodePtr )
      return (mIndex < _rhs.mIndex);

    return (mBodyNodePtr < _rhs.mBodyNodePtr);
  }

  /// Greater than
  template <class OtherDofT, class OtherBodyNodeT>
  bool operator > (const TemplateDegreeOfFreedomPtr<OtherDofT,
                   OtherBodyNodeT>& _rhs)
  {
    if( mBodyNodePtr == _rhs.mBodyNodePtr )
      return (mIndex > _rhs.mIndex);

    return (mBodyNodePtr > _rhs.mBodyNodePtr);
  }

  /// Less than or equal to
  template <class OtherDofT, class OtherBodyNodeT>
  bool operator <= (const TemplateDegreeOfFreedomPtr<OtherDofT,
                    OtherBodyNodeT>& _rhs)
  {
    return (*this < _rhs) || (*this == _rhs);
  }

  /// Greater than or equal to
  template <class OtherDofT, class OtherBodyNodeT>
  bool operator >= (const TemplateDegreeOfFreedomPtr<OtherDofT,
                    OtherBodyNodeT>& _rhs)
  {
    return (*this > _rhs) || (*this == _rhs);
  }

  /// \}

private:
  /// Reference-holding pointer to the child BodyNode of this DegreeOfFreedom
  TemplateBodyNodePtr<BodyNodeT> mBodyNodePtr;

  /// Local index of this DegreeOfFreedom within its Joint
  std::size_t mIndex;
};

/// TemplateWeakDegreeOfFreedomPtr is a templated class that enables users to
/// create a non-reference-holding WeakDegreeOfFreedomPtr. Holding onto a
/// WeakDegreeOfFreedomPtr will NOT prevent anything from getting deleted, but
/// you can use lock() to check whether the DegreeOfFreedom still exists. If it
/// does exist, it will return a valid strong DegreeOfFreedomPtr. Otherwise it
/// will return a nullptr DegreeOfFreedomPtr.
template <class DegreeOfFreedomT, class BodyNodeT>
class TemplateWeakDegreeOfFreedomPtr
{
public:

  template<class, class> friend class TemplateWeakDegreeOfFreedomPtr;

  /// Default constructor
  TemplateWeakDegreeOfFreedomPtr() { set(nullptr); }

  /// Typical constructor. _ptr must be a valid pointer (or a nullptr) when
  /// passed to this constructor
  TemplateWeakDegreeOfFreedomPtr(DegreeOfFreedomT* _ptr) { set(_ptr); }

  /// Constructor that takes in a WeakDegreeOfFreedomPtr
  template <class OtherDofT, class OtherBodyNodeT>
  TemplateWeakDegreeOfFreedomPtr(
      const TemplateWeakDegreeOfFreedomPtr<OtherDofT,
      OtherBodyNodeT>& _weakPtr)
  {
    set(_weakPtr);
  }

  /// Constructor that takes in a strong DegreeOfFreedomPtr
  template <class OtherDofT, class OtherBodyNodeT>
  TemplateWeakDegreeOfFreedomPtr(
      const TemplateDegreeOfFreedomPtr<OtherDofT,
      OtherBodyNodeT>& _strongPtr)
  {
    set(_strongPtr.get());
  }

  /// Assignment operator for raw DegreeOfFreedom pointers
  TemplateWeakDegreeOfFreedomPtr& operator = (DegreeOfFreedomT* _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Assignemnt operator for WeakDegreeOfFreedomPtrs
  template <class OtherDofT, class OtherBodyNodeT>
  TemplateWeakDegreeOfFreedomPtr& operator = (
      const TemplateWeakDegreeOfFreedomPtr<OtherDofT,
      OtherBodyNodeT>& _weakPtr)
  {
    set(_weakPtr);
    return *this;
  }

  /// Assignment operator for strong DegreeOfFreedomPtrs
  template <class OtherDofT, class OtherBodyNodeT>
  TemplateWeakDegreeOfFreedomPtr& operator = (
      const TemplateDegreeOfFreedomPtr<OtherDofT,
      OtherBodyNodeT>& _strongPtr)
  {
    set(_strongPtr.get());
    return *this;
  }

  /// Locks the DegreeOfFreedom reference to ensure that the referenced
  /// DegreeOfFreedom is currently still available. If the DegreeOfFreedom
  /// is not available any longer (i.e. has been deleted), then this will return
  /// a nullptr.
  TemplateDegreeOfFreedomPtr<DegreeOfFreedomT, BodyNodeT> lock() const
  {
    TemplateBodyNodePtr<BodyNodeT> bodyNode = mWeakBodyNode.lock();
    if(nullptr == bodyNode)
      return nullptr;

    return TemplateDegreeOfFreedomPtr<DegreeOfFreedomT, BodyNodeT>(
          bodyNode->getParentJoint()->getDof(mIndex));
  }

  /// Set the DegreeOfFreedom for this WeakDegreeOfFreedomPtr
  void set(DegreeOfFreedomT* _ptr)
  {
    if(nullptr == _ptr)
    {
      mWeakBodyNode = nullptr;
      mIndex = 0;
      return;
    }

    mWeakBodyNode = _ptr->getChildBodyNode();
    mIndex = _ptr->getIndexInJoint();
  }

  /// Attempt to set the DegreeOfFreedom for this WeakDegreeOfFreedomPtr based
  /// on another WeakDegreeOfFreedomPtr
  template <class OtherDofT, class OtherBodyNodeT>
  void set(const TemplateWeakDegreeOfFreedomPtr<OtherDofT,
           OtherBodyNodeT>& _weakPtr)
  {
    mWeakBodyNode = _weakPtr.mWeakBodyNode;
    mIndex = _weakPtr.mIndex;
  }

private:
  /// Weak pointer to the child BodyNode of this DegreeOfFreedom
  TemplateWeakBodyNodePtr<BodyNodeT> mWeakBodyNode;

  /// Local index of this DegreeOfFreedom within its Joint
  std::size_t mIndex;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_DEGREEOFFREEDOMPTR_HPP_
