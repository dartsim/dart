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

#ifndef DART_COMMON_LOCKABLEREFERENCE_IMPL_HPP_
#define DART_COMMON_LOCKABLEREFERENCE_IMPL_HPP_

#include "dart/common/LockableReference.hpp"

namespace dart {
namespace common {

//==============================================================================
template <typename Lockable>
SingleLockableReference<Lockable>::SingleLockableReference(
    std::weak_ptr<const void> lockableHolder, Lockable& lockable) noexcept
  : mLockableHolder(std::move(lockableHolder)), mLockable(lockable)
{
  // Do nothing
}

//==============================================================================
template <typename Lockable>
void SingleLockableReference<Lockable>::lock()
{
  if (mLockableHolder.expired())
    return;

  mLockable.lock();
}

//==============================================================================
template <typename Lockable>
bool SingleLockableReference<Lockable>::try_lock() noexcept
{
  if (mLockableHolder.expired())
    return false;

  return mLockable.try_lock();
}

//==============================================================================
template <typename Lockable>
void SingleLockableReference<Lockable>::unlock() noexcept
{
  if (mLockableHolder.expired())
    return;

  mLockable.unlock();
}

//==============================================================================
template <typename Lockable>
template <typename InputIterator>
MultiLockableReference<Lockable>::MultiLockableReference(
    std::weak_ptr<const void> lockableHolder,
    InputIterator first,
    InputIterator last)
  : mLockableHolder(std::move(lockableHolder)), mLockables(first, last)
{
  using IteratorValueType =
      typename std::iterator_traits<InputIterator>::value_type;
  using IteratorLockable = typename std::remove_pointer<
      typename std::remove_reference<IteratorValueType>::type>::type;

  static_assert(
      std::is_same<Lockable, IteratorLockable>::value,
      "Lockable of this class and the lockable of InputIterator are not the "
      "same.");
}

//==============================================================================
template <typename Lockable>
void MultiLockableReference<Lockable>::lock()
{
  if (mLockableHolder.expired())
    return;

  for (auto lockable : mLockables)
    lockable->lock();
}

//==============================================================================
template <typename Lockable>
bool MultiLockableReference<Lockable>::try_lock() noexcept
{
  if (mLockableHolder.expired())
    return false;

  for (auto lockable : mLockables)
  {
    if (!lockable->try_lock())
      return false;
  }

  return true;
}

//==============================================================================
template <typename Lockable>
void MultiLockableReference<Lockable>::unlock() noexcept
{
  if (mLockableHolder.expired())
    return;

  for (auto it = mLockables.rbegin(); it != mLockables.rend(); ++it)
    (*it)->unlock();
}

//==============================================================================
template <typename Lockable>
template <typename T>
T* MultiLockableReference<Lockable>::ptr(T& obj)
{
  return &obj;
}

//==============================================================================
template <typename Lockable>
template <typename T>
T* MultiLockableReference<Lockable>::ptr(T* obj)
{
  return obj;
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_LOCKABLEREFERENCE_IMPL_HPP_
