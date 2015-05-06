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
#ifndef DART_DYNAMICS_BODYNODEPTR_H_
#define DART_DYNAMICS_BODYNODEPTR_H_

#include <memory>
#include <mutex>

#include "dart/dynamics/Skeleton.h"

namespace dart{
namespace dynamics {

/// TemplateBodyNodePtr is a templated class enables users to create a
/// reference-counting BodyNodePtr. Holding onto a BodyNodePtr will ensure that
/// the BodyNode (and by extension, its Skeleton) does not get deleted. This
/// remains true even if the BodyNode is moved into another Skeleton.
template <class BodyNodeT>
class TemplateBodyNodePtr
{
public:
  /// Default constructor
  TemplateBodyNodePtr() : mPtr(nullptr) { }

  /// Typical constructor. _ptr must be a valid pointer when passed to this
  /// constructor
  TemplateBodyNodePtr(BodyNodeT* _ptr) : mPtr(nullptr) { set(_ptr); }

  /// Templated constructor for copying other BodyNodePtrs
  template <class OtherBodyNodeT>
  TemplateBodyNodePtr(const TemplateBodyNodePtr<OtherBodyNodeT>& _bnp)
    : mPtr(nullptr)
  {
    set(_bnp.get());
  }

  /// Destructor. Releases the BodyNode reference before being destroyed
  ~TemplateBodyNodePtr() { set(nullptr); }

  /// Change the BodyNode that this BodyNodePtr references
  template <class OtherBodyNodeT>
  TemplateBodyNodePtr& operator = (
      const TemplateBodyNodePtr<OtherBodyNodeT>& _bnp)
  {
    set(_bnp.get());
    return *this;
  }

  /// Assignment operator
  TemplateBodyNodePtr& operator = (BodyNodeT* _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Implicit conversion
  operator BodyNodeT*() const { return mPtr; }

  /// Dereferencing operator
  BodyNodeT& operator*() const { return *mPtr; }

  /// Dereferencing operation
  BodyNodeT* operator->() const { return mPtr; }

  /// Get the raw BodyNode pointer
  BodyNodeT* get() const { return mPtr; }

  /// Set the BodyNode for this BodyNodePtr
  void set(BodyNodeT* _ptr)
  {
    if(mPtr == _ptr)
      return;

    // Get a shared_ptr to each Skeleton before making any modifications to the
    // reference counts
    ConstSkeletonPtr old_skeleton = mPtr == nullptr ?
          nullptr : mPtr->getSkeleton();

    ConstSkeletonPtr new_skeleton = _ptr == nullptr ?
          nullptr : _ptr->getSkeleton();

    if(nullptr != mPtr)
    {
      if(nullptr != old_skeleton)
        mPtr->decrementReferenceCount();

      mPtr = nullptr;
    }

    if(nullptr != new_skeleton)
    {
      mPtr = _ptr;
      _ptr->incrementReferenceCount();
    }
  }

private:
  /// Raw pointer for the BodyNode that this BodyNodePtr references
  BodyNodeT* mPtr;
};

//==============================================================================
struct MutexedWeakSkeletonPtr
{
  std::mutex mMutex;
  std::weak_ptr<const Skeleton> mSkeleton;
};

//==============================================================================
template <class BodyNodeT>
class TemplateWeakBodyNodePtr
{
public:
  /// Default constructor
  TemplateWeakBodyNodePtr() : mPtr(nullptr) { }

  /// Typical constructor. _ptr must be a valid pointer when passed to this
  /// constructor
  TemplateWeakBodyNodePtr(BodyNodeT* _ptr) : mPtr(nullptr) { set(_ptr); }

  /// Constructor that takes in a WeakBodyNodePtr
  TemplateWeakBodyNodePtr(const TemplateWeakBodyNodePtr& _weakPtr) :
    mPtr(nullptr) { set(_weakPtr); }

  /// Assignment operator for raw BodyNode pointers
  TemplateWeakBodyNodePtr& operator = (BodyNodeT* _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Assignment operator for WeakBodyNodePtrs
  TemplateWeakBodyNodePtr& operator = (
      const TemplateWeakBodyNodePtr<BodyNodeT>& _weakPtr)
  {
    set(_weakPtr);
    return *this;
  }

  /// Locks the BodyNode reference to ensure that the referenced BodyNode (1) is
  /// currently still available, and (2) does not get deleted. If the BodyNode
  /// is not available any longer (i.e. has been deleted), then this will return
  /// a nullptr.
  ///
  /// To keep the BodyNode active, you should capture the return value of this
  /// function in a BodyNodePtr.
  TemplateBodyNodePtr<BodyNodeT> lock() const
  {
    if(nullptr == mLocker)
      return nullptr;

    // We do not use the expired() function here, because we want to ensure that
    // the Skeleton's reference count remains up while we create the strong
    // BodyNodePtr that we're going to return.
    std::lock_guard<std::mutex> lock(mLocker->mMutex);
    ConstSkeletonPtr skeleton = mLocker->mSkeleton.lock();
    if(nullptr == skeleton)
      return nullptr;

    return TemplateBodyNodePtr<BodyNodeT>(mPtr);
  }

  /// Set the BodyNode for this WeakBodyNodePtr
  void set(BodyNodeT* _ptr)
  {
    mPtr = _ptr;

    if(nullptr == mPtr)
      mLocker = nullptr;
    else
      mLocker = _ptr->mLockedSkeleton;
  }

  /// Attempt to set the BodyNode for this WeakBodyNodePtr based on another
  /// WeakBodyNodePtr
  void set(const TemplateWeakBodyNodePtr& _weakPtr)
  {
    if(nullptr == _weakPtr.mLocker)
    {
      set(nullptr);
      return;
    }

    std::lock_guard<std::mutex> lock(_weakPtr.mLocker->mMutex);
    ConstSkeletonPtr skeleton = _weakPtr.mLocker->mSkeleton.lock();
    if(nullptr == skeleton)
    {
      set(nullptr);
      return;
    }

    set(_weakPtr.mPtr);
  }

  /// Returns true if this WeakBodyNodePtr is referencing a nullptr or a pointer
  /// which has been deleted. Returns false if it is referencing a pointer which
  /// is still active.
  ///
  /// Note: in multithreaded application, there is no guarantee that the pointer
  /// will still be active after this function has finished. To guarantee that
  /// the pointer remains active, use lock() and store its return in a
  /// BodyNodePtr
  bool expired() const
  {
    if(nullptr == mLocker)
      return true;

    // It is okay for 'lock' to go "unused", because it is managed by RAII after
    // it has been initialized
    std::lock_guard<std::mutex> lock(mLocker->mMutex);
    ConstSkeletonPtr skeleton = mLocker->mSkeleton.lock();
    if(nullptr == skeleton)
      return true;

    return false;
  }

private:
  /// Raw pointer for the BodyNode that this WeakBodyNodePtr references
  BodyNodeT* mPtr;

  /// A shared_ptr that allows the WeakBodyNodePtr to know whether it can lock
  /// into a BodyNodePtr
  std::shared_ptr<MutexedWeakSkeletonPtr> mLocker;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_BODYNODEPTR_H_
