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

#ifndef DART_DYNAMICS_DETAIL_BODYNODEPTR_HPP_
#define DART_DYNAMICS_DETAIL_BODYNODEPTR_HPP_

#include <memory>
#include <mutex>
#include <atomic>

namespace dart{
namespace dynamics {

class Skeleton;

//==============================================================================
struct MutexedWeakSkeletonPtr
{
  std::mutex mMutex;
  std::weak_ptr<const Skeleton> mSkeleton;
};

//==============================================================================
class SkeletonRefCountingBase
{
public:
  template<class> friend class TemplateBodyNodePtr;
  template<class> friend class TemplateWeakBodyNodePtr;

  /// Return the Skeleton this BodyNode belongs to
  std::shared_ptr<Skeleton> getSkeleton();

  /// Return the (const) Skeleton this BodyNode belongs to
  std::shared_ptr<const Skeleton> getSkeleton() const;

private:

  //--------------------------------------------------------------------------
  // Reference counting
  //--------------------------------------------------------------------------

  /// Atomically increment the reference count for this BodyNode. This should
  /// only be called by the BodyNodePtr class
  void incrementReferenceCount() const;

  /// Atomically decrement the reference count for this BodyNode. This should
  /// only be called by the BodyNodePtr class
  void decrementReferenceCount() const;

protected:

  /// Default Constructor
  SkeletonRefCountingBase();

  /// Weak pointer to the Skeleton this BodyNode belongs to.
  std::weak_ptr<Skeleton> mSkeleton;

  /// Reference count for the number of BodyNodePtrs that are referring to this
  /// BodyNode
  mutable std::atomic<int> mReferenceCount;

  /// If mReferenceCount is zero, then mReferenceSkeleton will hold a nullptr.
  /// If mReferenceCount is greater than zero, then mReferenceSkeleton will hold
  /// a shared_ptr to the Skeleton that this BodyNode belongs to. This is to
  /// keep this BodyNode alive, so long as a BodyNodePtr that references it
  /// exists.
  mutable std::shared_ptr<Skeleton> mReferenceSkeleton;

  /// Shared reference to a weak_ptr of this BodyNode's Skeleton, along with a
  /// mutex to ensure thread safety. This is used by WeakBodyNodePtrs to know
  /// when this BodyNode has expired.
  std::shared_ptr<MutexedWeakSkeletonPtr> mLockedSkeleton;

};

/// TemplateBodyNodePtr is a templated class that enables users to create a
/// reference-counting BodyNodePtr. Holding onto a BodyNodePtr will ensure that
/// the BodyNode (and by extension, its Skeleton) does not get deleted. This
/// remains true even if the BodyNode is moved into another Skeleton.
template <class BodyNodeT>
class TemplateBodyNodePtr
{
public:
  /// Default constructor
  TemplateBodyNodePtr() : mPtr(nullptr) { }

  /// Typical constructor. _ptr must be a valid pointer (or a nullptr) when
  /// passed to this constructor
  TemplateBodyNodePtr(BodyNodeT* _ptr) : mPtr(nullptr) { set(_ptr); }

  /// User defined copy-constructor
  TemplateBodyNodePtr(const TemplateBodyNodePtr& _bnp)
    : mPtr(nullptr)
  {
    set(_bnp.get());
  }

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

    if(nullptr != mPtr)
    {
      static_cast<const SkeletonRefCountingBase*>(mPtr)->
          decrementReferenceCount();
    }

    if(nullptr != _ptr)
    {
      static_cast<const SkeletonRefCountingBase*>(_ptr)->
          incrementReferenceCount();
    }

    mPtr = _ptr;
  }

private:
  /// Raw pointer for the BodyNode that this BodyNodePtr references
  BodyNodeT* mPtr;
};

/// TemplateWeakBodyNodePtr is a templated class that enables users to create a
/// non-reference-holding WeakBodyNodePtr. Holding onto a WeakBodyNodePtr will
/// NOT prevent the BodyNode from getting deleted, but you can use lock() to
/// check whether the BodyNode still exists. If it does exist, it will return a
/// valid strong BodyNodePtr. Otherwise, it will return a nullptr BodyNodePtr.
template <class BodyNodeT>
class TemplateWeakBodyNodePtr
{
public:

  template<class> friend class TemplateWeakBodyNodePtr;

  /// Default constructor
  TemplateWeakBodyNodePtr() : mPtr(nullptr) { }

  /// Typical constructor. _ptr must be a valid pointer (or a nullptr) when
  /// passed to this constructor
  TemplateWeakBodyNodePtr(BodyNodeT* _ptr) : mPtr(nullptr) { set(_ptr); }

  /// Constructor that takes in a WeakBodyNodePtr
  template <class OtherBodyNodeT>
  TemplateWeakBodyNodePtr(
      const TemplateWeakBodyNodePtr<OtherBodyNodeT>& _weakPtr)
    : mPtr(nullptr) { set(_weakPtr); }

  /// Assignment operator for raw BodyNode pointers
  TemplateWeakBodyNodePtr& operator = (BodyNodeT* _ptr)
  {
    set(_ptr);
    return *this;
  }

  /// Assignment operator for WeakBodyNodePtrs
  template <class OtherBodyNodeT>
  TemplateWeakBodyNodePtr& operator = (
      const TemplateWeakBodyNodePtr<OtherBodyNodeT>& _weakPtr)
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
    std::shared_ptr<const Skeleton> skeleton = mLocker->mSkeleton.lock();
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
      mLocker = static_cast<const SkeletonRefCountingBase*>(_ptr)->
          mLockedSkeleton;
  }

  /// Attempt to set the BodyNode for this WeakBodyNodePtr based on another
  /// WeakBodyNodePtr
  template <class OtherBodyNodeT>
  void set(const TemplateWeakBodyNodePtr<OtherBodyNodeT>& _weakPtr)
  {
    if(nullptr == _weakPtr.mLocker)
    {
      set(nullptr);
      return;
    }

    std::lock_guard<std::mutex> lock(_weakPtr.mLocker->mMutex);
    std::shared_ptr<const Skeleton> skeleton =
        _weakPtr.mLocker->mSkeleton.lock();
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
    std::shared_ptr<const Skeleton> skeleton = mLocker->mSkeleton.lock();
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

#endif // DART_DYNAMICS_DETAIL_BODYNODEPTR_HPP_
