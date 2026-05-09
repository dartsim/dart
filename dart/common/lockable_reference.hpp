/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#ifndef DART_COMMON_LOCKABLEREFERENCE_HPP_
#define DART_COMMON_LOCKABLEREFERENCE_HPP_

#include <concepts>
#include <iterator>
#include <memory>
#include <type_traits>
#include <vector>

namespace dart {
namespace common {

namespace detail {

template <typename T>
concept LockableObject = requires(T lockable) {
  { lockable.lock() } -> std::same_as<void>;
  { lockable.try_lock() } -> std::convertible_to<bool>;
  { lockable.unlock() } -> std::same_as<void>;
};

template <typename Iterator, typename Lockable>
concept LockablePointerInputIterator
    = std::input_iterator<Iterator>
      && std::same_as<
          std::remove_pointer_t<std::remove_cvref_t<
              typename std::iterator_traits<Iterator>::value_type>>,
          Lockable>;

} // namespace detail

/// LockableReference is a wrapper class of single or multiple Lockable
/// object(s) to provide unified interface that guarantees deadlock-free locking
/// and unlocking of the internal lockable(s).
///
/// This class is compatible to BasicLockable concept so that it can be used
/// as a template parameter that requires BasicLockable concept such as
/// std::lock_guard.
class LockableReference
{
public:
  /// Default constructor
  constexpr LockableReference() noexcept = default;

  /// Default destructor
  virtual ~LockableReference() = default;

  /// Locks lockable that this class references; blocks if one of the lockables
  /// are
  /// not available.
  virtual void lock() = 0;

  /// Tries to lock the lockables that this class references; returns false if
  /// one of the lockables is not available.
  virtual bool try_lock() noexcept = 0;

  /// Unlocks the lockables.
  virtual void unlock() noexcept = 0;

protected:
  /// Copy construction is not allowed.
  LockableReference(const LockableReference&) = delete;
};

/// This class references a single lockable.
///
/// @tparam LockableT The standard C++ Lockable concept object type.
template <detail::LockableObject LockableT>
class SingleLockableReference final : public LockableReference
{
public:
  using Lockable = LockableT;

  /// Constructor from a single lockable.
  ///
  /// @param[in] lockableHolder Weak pointer to an object that holds the
  /// lockable. This is used to check whether the lockable holder is not
  /// destructed before lock/unlock.
  /// @param[in] lockable
  SingleLockableReference(
      std::weak_ptr<const void> lockableHolder, Lockable& lockable) noexcept;

  // Documentation inherited
  void lock() override;

  // Documentation inherited
  bool try_lock() noexcept override;

  // Documentation inherited
  void unlock() noexcept override;

private:
  /// Weak pointer to the lockable holder.
  std::weak_ptr<const void> mLockableHolder;

  /// Lockable this class references.
  Lockable& mLockable;
};

/// MultiLockableReference references multiple lockables.
///
/// MultiLockableReference acquires the locks in the specified order, which
/// means it is the user's responsibility to sort the collection to avoid
/// deadlock.
///
/// @tparam LockableT The standard C++ Lockable concept object type.
template <detail::LockableObject LockableT>
class MultiLockableReference final : public LockableReference
{
public:
  using Lockable = LockableT;

  /// Constructs from multiple lockables.
  ///
  /// @param[in] lockableHolder Weak pointer to an object that holds the
  /// lockables. This is used to lock/unlock this lockable only when the
  /// lockable holder is not destructed.
  /// @param[in] first First iterator of lockable to be added to this class.
  /// @param[in] last Last iterator of lockable to be added to this class.
  template <typename InputIterator>
    requires detail::LockablePointerInputIterator<InputIterator, Lockable>
  MultiLockableReference(
      std::weak_ptr<const void> lockableHolder,
      InputIterator first,
      InputIterator last);

  // Documentation inherited
  void lock() override;

  // Documentation inherited
  bool try_lock() noexcept override;

  // Documentation inherited
  void unlock() noexcept override;

private:
  /// Converts reference to pointer.
  template <typename T>
  T* ptr(T& obj);

  /// Returns pointer as it is.
  template <typename T>
  T* ptr(T* obj);

  /// Weak pointer to the lockable holder.
  std::weak_ptr<const void> mLockableHolder;

  /// lockables this class references.
  std::vector<Lockable*> mLockables;
};

} // namespace common
} // namespace dart

#include <dart/common/detail/lockable_reference-impl.hpp>

#endif // DART_COMMON_LOCKABLEREFERENCE_HPP_
