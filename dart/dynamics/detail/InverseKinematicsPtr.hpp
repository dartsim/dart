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

#ifndef DART_DYNAMICS_DETAIL_INVERSEKINEMATICSPTR_HPP_
#define DART_DYNAMICS_DETAIL_INVERSEKINEMATICSPTR_HPP_

#include <memory>
#include "dart/dynamics/detail/NodePtr.hpp"

namespace dart {
namespace dynamics {

/// TemplateInverseKinematicsPtr is a templated class that enables users to
/// create a reference-counting InverseKinematicsPtr. Holding onto an
/// InverseKinematicsPtr will ensure that the JacobianNode associated with the
/// InverseKinematics module will not get deleted, and will keep the
/// InverseKinematics reference valid.
template <class IkType, class JacobianNodePtrT>
class TemplateInverseKinematicsPtr
{
public:

  template<class, class> friend class TemplateInverseKinematicsPtr;

  typedef IkType element_type;

  /// Constructor that accepts a shared_ptr
  TemplateInverseKinematicsPtr(const std::shared_ptr<element_type>& sptr)
  {
    set(sptr);
  }

  /// Constructor that accepts a nullptr
  TemplateInverseKinematicsPtr(std::nullptr_t)
  {
    // Do nothing
  }

  /// Default constructor
  TemplateInverseKinematicsPtr() = default;

  /// Constructor that takes in a strong InverseKinematicsPtr
  template <class OtherIkT, class OtherJacNodePtrT>
  TemplateInverseKinematicsPtr(
      const TemplateInverseKinematicsPtr<OtherIkT, OtherJacNodePtrT>& _ptr)
  {
    set(_ptr.mIK);
  }

  /// Implicit conversion to regular shared_ptr
  operator std::shared_ptr<element_type>() const { return mIK; }

  /// Constructor that takes in a shared_ptr
  template <class OtherIkT>
  TemplateInverseKinematicsPtr(const std::shared_ptr<OtherIkT>& sptr)
  {
    set(sptr);
  }

  /// Assignment operator
  template <class OtherIkT, class OtherJacNodePtrT>
  TemplateInverseKinematicsPtr& operator = (
      const TemplateInverseKinematicsPtr<OtherIkT, OtherJacNodePtrT>& _ptr)
  {
    set(_ptr.mIK);
    return *this;
  }

  /// Assignment operator for shared_ptr
  template <class SharedPtrT>
  TemplateInverseKinematicsPtr& operator = (const SharedPtrT& _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Assignment operator for nullptr
  TemplateInverseKinematicsPtr& operator = (std::nullptr_t)
  {
    set(nullptr);
    return *this;
  }

  /// Implicit conversion to bool
  operator bool() const { return (nullptr != mIK); }

  /// Dereferencing operator
  element_type& operator*() const { return *get(); }

  /// Dereferencing operation
  element_type* operator->() const { return get(); }

  /// Get the raw pointer
  element_type* get() const
  {
    if(nullptr == mJacNodePtr)
      return nullptr;

    return mIK.get();
  }

  /// Get the shared_ptr held by this InverseKinematicsPtr
  std::shared_ptr<element_type> get_shared() const
  {
    if(nullptr == mJacNodePtr)
      return nullptr;

    return mIK;
  }

  /// Set the InverseKinematics module for this InverseKinematicsPtr from a
  /// shared_ptr
  void set(const std::shared_ptr<IkType>& sptr)
  {
    if(nullptr == sptr)
    {
      mIK = nullptr;
      mJacNodePtr = nullptr;
      return;
    }

    mJacNodePtr = sptr->getAffiliation();
    mIK = sptr;
  }

  //----------------------------------------------------------------------------
  /// \{ \name Comparison operators
  //----------------------------------------------------------------------------

  /// Equality
  template <class OtherIkT, class OtherJacNodeT>
  bool operator == (
      const TemplateInverseKinematicsPtr<OtherIkT, OtherJacNodeT>& _rhs)
  {
    return (mIK == _rhs.mIK);
  }

  /// Inequality
  template <class OtherIkT, class OtherJacNodeT>
  bool operator != (
      const TemplateInverseKinematicsPtr<OtherIkT, OtherJacNodeT>& _rhs)
  {
    return !( *this == _rhs );
  }

  /// Less than
  template <class OtherIkT, class OtherJacNodeT>
  bool operator < (
      const TemplateInverseKinematicsPtr<OtherIkT, OtherJacNodeT>& _rhs)
  {
    return (mIK < _rhs.mIK);
  }

  /// Greater than
  template <class OtherIkT, class OtherJacNodeT>
  bool operator > (
      const TemplateInverseKinematicsPtr<OtherIkT, OtherJacNodeT>& _rhs)
  {
    return (mIK > _rhs.mIK);
  }

  /// Less than or equal to
  template <class OtherIkT, class OtherJacNodeT>
  bool operator <= (
      const TemplateInverseKinematicsPtr<OtherIkT, OtherJacNodeT>& _rhs)
  {
    return (*this < _rhs) || (*this == _rhs);
  }

  /// Greater than or equal to
  template <class OtherIkT, class OtherJacNodeT>
  bool operator >= (
      const TemplateInverseKinematicsPtr<OtherIkT, OtherJacNodeT>& _rhs)
  {
    return (*this > _rhs) || (*this == _rhs);
  }

  /// \}

protected:

  /// Pointer to the IK module
  std::shared_ptr<element_type> mIK;

  /// Pointer to the Node associated with the IK module
  JacobianNodePtrT mJacNodePtr;

};

// Comparison to nullptr
template <class IkType, class BodyNodeT>
inline bool operator == (
    const TemplateInverseKinematicsPtr<IkType, BodyNodeT>& _ik, std::nullptr_t)
{
  return nullptr == _ik.get();
}

// Comparison to nullptr
template <class IkType, class BodyNodeT>
inline bool operator == (
    std::nullptr_t, const TemplateInverseKinematicsPtr<IkType, BodyNodeT>& _ik)
{
  return nullptr == _ik.get();
}

// Comparison to nullptr
template <class IkType, class BodyNodeT>
inline bool operator != (
    const TemplateInverseKinematicsPtr<IkType, BodyNodeT>& _ik, std::nullptr_t)
{
  return nullptr != _ik.get();
}

// Comparison to nullptr
template <class IkType, class BodyNodeT>
inline bool operator != (
    std::nullptr_t, const TemplateInverseKinematicsPtr<IkType, BodyNodeT>& _ik)
{
  return nullptr != _ik.get();
}


/// TemplateWeakInverseKinematicsPtr is a templated class that enables users to
/// create a non-reference-holding WeakInverseKinematicsPtr. Holding onto a
/// WeakInverseKinematicsPtr will NOT prevent anything from getting deleted, but
/// you can use lock() to check whether the InverseKinematics module and its
/// associated JacobianNode still exists.
template <class InverseKinematicsT, class JacobianNodePtrT>
class TemplateWeakInverseKinematicsPtr
{
public:

  template<class, class> friend class TemplateWeakInverseKinematicsPtr;

  typedef InverseKinematicsT element_type;

  /// Default constructor
  TemplateWeakInverseKinematicsPtr() = default;

  /// Constructor for various pointer types
  template <class PtrType>
  TemplateWeakInverseKinematicsPtr(const PtrType& _ptr)
  {
    set(_ptr);
  }

  /// Assignment operator for various templated pointer types
  template <class PtrType>
  TemplateWeakInverseKinematicsPtr& operator = (const PtrType& _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Locks the InverseKinematics module to ensure that the referenced module is
  /// currently still available. If the module is not available any longer (i.e.
  /// has been deleted), then this will return a nullptr.
  TemplateInverseKinematicsPtr<InverseKinematicsT, JacobianNodePtrT> lock() const
  {
    JacobianNodePtrT jacNode = mWeakJacNode.lock();
    if(nullptr == jacNode)
      return nullptr;

    return TemplateInverseKinematicsPtr<InverseKinematicsT, JacobianNodePtrT>(
          mWeakIK.lock());
  }

  /// Set using a strong pointer
  template <class OtherIkT, class OtherJacNodePtrT>
  void set(const TemplateInverseKinematicsPtr<OtherIkT, OtherJacNodePtrT>& _ptr)
  {
    if(nullptr == _ptr)
    {
      mWeakIK = nullptr;
      mWeakJacNode = nullptr;
      return;
    }

    mWeakIK = _ptr.get();
    mWeakJacNode = _ptr.get()->getEntity();
  }

  /// Set using a weak pointer
  template <class OtherIkT, class OtherJacNodeT>
  void set(const TemplateWeakInverseKinematicsPtr<
           OtherIkT, OtherJacNodeT>& _ptr)
  {
    mWeakIK = _ptr.mWeakIK;
    mWeakJacNode = _ptr.mWeakJacNode;
  }

protected:

  /// Weak pointer to the IK module
  std::weak_ptr<InverseKinematicsT> mWeakIK;

  /// Weak pointer to the JacobianNode
  JacobianNodePtrT mWeakJacNode;

};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_INVERSEKINEMATICSPTR_HPP_
