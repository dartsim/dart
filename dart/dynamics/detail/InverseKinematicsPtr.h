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

#ifndef DART_DYNAMICS_DETAIL_INVERSEKINEMATICSPTR_H_
#define DART_DYNAMICS_DETAIL_INVERSEKINEMATICSPTR_H_

#include <memory>
#include "dart/dynamics/detail/NodePtr.h"

namespace dart {
namespace dynamics {

/// TemplateInverseKinematicsPtr is a templated class that enables users to
/// create a reference-counting InverseKinematicsPtr. Holding onto an
/// InverseKinematicsPtr will ensure that the JacobianNode associated with the
/// InverseKinematics module will not get deleted, and will keep the
/// InverseKinematics reference valid.
template <class InverseKinematicsT, class JacobianNodePtrT>
class TemplateInverseKinematicsPtr
{
public:

  template<class, class> friend class TemplateInverseKinematicsPtr;

  typedef InverseKinematicsT element_type;

  /// Constructor that accepts a shared_ptr
  TemplateInverseKinematicsPtr(const std::shared_ptr<element_type>& _sptr)
  {
    set(_sptr);
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
  TemplateInverseKinematicsPtr(const std::shared_ptr<OtherIkT>& _sptr)
  {
    set(_sptr);
  }

  /// Assignment operator
  template <class OtherIkT, class OtherJacNodePtrT>
  TemplateInverseKinematicsPtr& operator = (
      const TemplateInverseKinematicsPtr<OtherIkT, OtherJacNodePtrT>& _ptr)
  {
    set(_ptr.mIK);
    return *this;
  }

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
  void set(const std::shared_ptr<InverseKinematicsT>& _sptr)
  {
    if(nullptr == _sptr)
    {
      mIK = nullptr;
      mJacNodePtr = nullptr;
      return;
    }

    mJacNodePtr = _sptr->getObject();
    mIK = _sptr;
  }

protected:

  std::shared_ptr<element_type> mIK;

  JacobianNodePtrT mJacNodePtr;

};

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

#endif // DART_DYNAMICS_DETAIL_INVERSEKINEMATICSPTR_H_
