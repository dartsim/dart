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

#ifndef DART_DYNAMICS_DETAIL_NODEPTR_HPP_
#define DART_DYNAMICS_DETAIL_NODEPTR_HPP_

#include "dart/dynamics/detail/BodyNodePtr.hpp"

namespace dart {
namespace dynamics {

class NodeDestructor;

/// TemplateNodePtr is a templated class that enables users to create a strong
/// reference-counting NodePtr. Holding onto a NodePtr will ensure that the
/// BodyNode associated with a Node does not get deleted, and it will also
/// ensure that the Node itself does not get deleted. This templated class can
/// be applied to any class that inherits from dart::dynamics::Node.
template <class NodeT, class BodyNodeT>
class TemplateNodePtr
{
public:

  template<class, class> friend class TemplateNodePtr;

  using element_type = NodeT;

  /// Default constructor
  TemplateNodePtr() : mNode(nullptr) { }

  /// Typical constructor. _ptr must be a valid pointer (or a nullptr) when
  /// passed to this constructor
  TemplateNodePtr(NodeT* _ptr) { set(_ptr); }

  /// Constructor that takes in a strong NodePtr
  template <class OtherNodeT, class OtherBodyNodeT>
  TemplateNodePtr(
      const TemplateNodePtr<OtherNodeT, OtherBodyNodeT>& _ptr)
  {
    set(_ptr.get());
  }

  /// Assignment operator
  TemplateNodePtr& operator = (NodeT* _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Assignment operator for NodePtrs
  template <class OtherNodeT, class OtherBodyNodeT>
  TemplateNodePtr& operator = (
      const TemplateNodePtr<OtherNodeT, OtherBodyNodeT>& _ptr)
  {
    set(_ptr.get());
    return *this;
  }

  /// Implicit conversion
  operator NodeT*() const { return get(); }

  /// Dereferencing operator
  NodeT& operator*() const { return *get(); }

  /// Dereferencing operation
  NodeT* operator->() const { return get(); }

  /// Get the raw Node pointer
  NodeT* get() const
  {
    return mNode;
  }

  /// Set the Node for this NodePtr
  void set(NodeT* _ptr)
  {
    if(nullptr == _ptr)
    {
      mBodyNodePtr = nullptr;
      mDestructor = nullptr;
      mNode = nullptr;
      return;
    }

    mBodyNodePtr = _ptr->getBodyNodePtr();
    mDestructor = _ptr->mDestructor.lock();
    mNode = _ptr;
  }

protected:

  /// Node that this NodePtr refers to
  NodeT* mNode;

  /// Hold onto a shared_ptr to the Node's Destructor to make sure the Node stays
  /// alive.
  std::shared_ptr<NodeDestructor> mDestructor;

  /// Hold onto a BodyNodePtr to the Node's associated BodyNode to make sure
  /// that the BodyNode stays alive.
  TemplateBodyNodePtr<BodyNodeT> mBodyNodePtr;
};

/// TemplateWeakNodePtr is a templated class that enables users to create a weak
/// non-reference-holding WeakNodePtr. Holding onto a WeakNodePtr will NOT
/// prevent anything from getting deleted, but you can use lock() to check
/// whether the Node still exists. If it does exist, it will return a valid
/// NodePtr. Otherwise it will return a nullptr NodePtr.
template <class NodeT, class BodyNodeT>
class TemplateWeakNodePtr
{
public:

  template <class, class> friend class TemplateWeakNodePtr;

  /// Default constructor
  TemplateWeakNodePtr() : mNode(nullptr) { }

  /// Typical constructor. _ptr must be a valid pointer (or a nullptr) when
  /// passed to this constructor
  TemplateWeakNodePtr(NodeT* _ptr) { set(_ptr); }

  /// Constructor that takes in a WeakNodePtr
  template <class OtherNodeT, class OtherBodyNodeT>
  TemplateWeakNodePtr(
      const TemplateWeakNodePtr<OtherNodeT, OtherBodyNodeT>& _weakPtr)
  {
    set(_weakPtr);
  }

  /// Constructor that takes in a strong NodePtr
  template <class OtherNodeT, class OtherBodyNodeT>
  TemplateWeakNodePtr(
      const TemplateNodePtr<OtherNodeT, OtherBodyNodeT>& _strongPtr)
  {
    set(_strongPtr.get());
  }

  /// Assignment operator for raw Node pointers
  TemplateWeakNodePtr& operator = (NodeT* _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Assignment operator for WeakNodePtrs
  template <class OtherNodeT, class OtherBodyNodeT>
  TemplateWeakNodePtr& operator = (
      const TemplateWeakNodePtr<OtherNodeT, OtherBodyNodeT>& _weakPtr)
  {
    set(_weakPtr);
    return *this;
  }

  /// Assignment operator for strong NodePtrs
  template <class OtherNodeT, class OtherBodyNodeT>
  TemplateWeakNodePtr& operator = (
      const TemplateNodePtr<OtherNodeT, OtherBodyNodeT>& _strongPtr)
  {
    set(_strongPtr.get());
    return *this;
  }

  /// Locks the Node reference to ensure that the referenced Node is currently
  /// still available. If the Node is not available any longer (i.e. has been
  /// deleted), then this will return a nullptr.
  TemplateNodePtr<NodeT, BodyNodeT> lock() const
  {
    TemplateBodyNodePtr<BodyNodeT> bodyNode = mWeakBodyNodePtr.lock();
    if(nullptr == bodyNode)
      return nullptr;

    std::shared_ptr<NodeDestructor> destructor = mWeakDestructor.lock();
    if(nullptr == destructor)
      return nullptr;

    return TemplateNodePtr<NodeT, BodyNodeT>(mNode);
  }

  /// Set the Node for this WeakNodePtr
  void set(NodeT* _ptr)
  {
    if(nullptr == _ptr)
    {
      mNode = nullptr;
      mWeakDestructor.reset();
      mWeakBodyNodePtr = nullptr;
      return;
    }

    mWeakBodyNodePtr = _ptr->getBodyNodePtr();
    mWeakDestructor = _ptr->getOrCreateDestructor();
    mNode = _ptr;
  }

  /// Set the Node for this WeakNodePtr based on another WeakNodePtr
  template <class OtherNodeT, class OtherBodyNodeT>
  void set(const TemplateWeakNodePtr<OtherNodeT, OtherBodyNodeT>& _weakPtr)
  {
    mNode = _weakPtr.mNode;
    mWeakDestructor = _weakPtr.mWeakDestructor;
    mWeakBodyNodePtr = _weakPtr.mWeakBodyNodePtr;
  }

protected:

  /// Node that this pointer references
  NodeT* mNode;

  /// Destructor for the Node
  std::weak_ptr<NodeDestructor> mWeakDestructor;

  /// Pointer to the BodyNode that the Node is attached to
  TemplateWeakBodyNodePtr<BodyNodeT> mWeakBodyNodePtr;
};



} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_NODEPTR_HPP_
