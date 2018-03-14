/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#ifndef DART_COMMON_MUTEXREFERENCE_HPP_
#define DART_COMMON_MUTEXREFERENCE_HPP_

#include <memory>
#include <mutex>
#include <vector>

namespace dart {
namespace common {

/// Mutex is a wrapper class of single or multiple std::mutex to provide unified
/// interface that guarantees deadlock-free locking and unlocking of the
/// internal mutex(es).
///
/// This class is compatible to BasicLockable concept so that it can be used
/// as a template parameter that requires this concept such as std::lock_guard.
class MutexReference
{
public:
  /// Default construtor
  constexpr MutexReference() noexcept = default;

  /// Default destructor
  virtual ~MutexReference() = default;

  /// Locks mutex that this class references; blocks if one of the mutexes are
  /// not avaliable.
  virtual void lock() = 0;

  /// Tries to lock the mutexes that this class references; returns false if one
  /// of the mutexes is not avaliable.
  virtual bool try_lock() noexcept = 0;

  /// Unlocks the mutexes.
  virtual void unlock() noexcept = 0;

protected:
  /// Copy construction is not allowed.
  MutexReference(const MutexReference&) = delete;
};

/// This class references a single mutex.
template <typename LockableT = std::mutex>
class SingleMutexReference final : public MutexReference
{
public:
  using Lockable = LockableT;

  /// Constructor from a single mutex.
  ///
  /// \param[in] mutexHolder Weak pointer to an object that holds the mutex.
  /// This is used to lock/unlock this lockable only when the mutex holder is
  /// not destructed.
  SingleMutexReference(
      std::weak_ptr<const void> mutexHolder, Lockable& mutex) noexcept;

  // Documentation inherited
  void lock() override;

  // Documentation inherited
  bool try_lock() noexcept override;

  // Documentation inherited
  void unlock() noexcept override;

private:
  /// Weak pointer to the mutex holder.
  std::weak_ptr<const void> mMutexHolder;

  /// Mutex this class references.
  Lockable& mMutex;
};

/// MultiMutexReference references multiple mutexes.
///
/// MultiMutexReference acquires the locks in the specified order, which means
/// it is the user's responsibility to sort the collection to avoid deadlock.
template <typename InputIteratorT>
class MultiMutexReference final : public MutexReference
{
public:
  using Iterator = InputIteratorT;

  /// Constructs from multiple mutexes.
  ///
  /// \param[in] mutexHolder Weak pointer to an object that holds the mutexes.
  /// This is used to lock/unlock this lockable only when the mutex holder is
  /// not destructed.
  /// \param[in] first First iterator of lockable to be added to this class.
  /// \param[in] last Last iterator of lockable to be added to this class.
  MultiMutexReference(
      std::weak_ptr<const void> mutexHolder, Iterator first, Iterator last);

  // Documentation inherited
  void lock() override;

  // Documentation inherited
  bool try_lock() noexcept override;

  // Documentation inherited
  void unlock() noexcept override;

private:
  using IteratorValueType = typename std::iterator_traits<Iterator>::value_type;
  using Lockable = typename std::remove_pointer<
      typename std::remove_reference<IteratorValueType>::type>::type;

  /// Converts reference to pointer.
  template <typename T>
  T* ptr(T& obj);

  /// Returns pointer as it is.
  template <typename T>
  T* ptr(T* obj);

  /// Weak pointer to the mutex holder.
  std::weak_ptr<const void> mMutexHolder;

  /// Mutexes this class references.
  std::vector<Lockable*> mMutexes;
};

} // namespace common
} // namespace dart

#include "dart/common/detail/MutexReference-impl.hpp"

#endif // DART_COMMON_MUTEXREFERENCE_HPP_
