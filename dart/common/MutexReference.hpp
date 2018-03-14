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
#include <set>

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

  /// Tries to lock the mutexes that this class references; returns false if
  /// one of the mutexes is not avaliable.
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
  LockableT& mMutex;
};

/// This class references multiple mutexes.
class MultiMutexReference final : public MutexReference
{
public:
  /// Constructs from multiple mutexes.
  MultiMutexReference(
      std::weak_ptr<const void> mutexHolder,
      const std::set<std::mutex*>& mutexes) noexcept;

  // Documentation inherited
  void lock() override;

  // Documentation inherited
  bool try_lock() noexcept override;

  // Documentation inherited
  void unlock() noexcept override;

private:
  /// Weak pointer to the mutex holder.
  std::weak_ptr<const void> mMutexHolder;

  /// Mutexes this class references.
  std::set<std::mutex*> mMutexes;
};

} // namespace common
} // namespace dart

#include "dart/common/detail/MutexReference-impl.hpp"

#endif // DART_COMMON_MUTEXREFERENCE_HPP_
