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

#ifndef DART_COMMON_MUTEXREFERENCE_IMPL_HPP_
#define DART_COMMON_MUTEXREFERENCE_IMPL_HPP_

#include "dart/common/MutexReference.hpp"

namespace dart {
namespace common {

//==============================================================================
template <typename Lockable>
SingleMutexReference<Lockable>::SingleMutexReference(
    std::weak_ptr<const void> mutexHolder, Lockable& mutex) noexcept
    : mMutexHolder(std::move(mutexHolder)),
      mMutex(mutex)
{
  // Do nothing
}

//==============================================================================
template <typename Lockable>
void SingleMutexReference<Lockable>::lock()
{
  if (mMutexHolder.expired())
    return;

  mMutex.lock();
}

//==============================================================================
template <typename Lockable>
bool SingleMutexReference<Lockable>::try_lock() noexcept
{
  if (mMutexHolder.expired())
    return false;

  return mMutex.try_lock();
}

//==============================================================================
template <typename Lockable>
void SingleMutexReference<Lockable>::unlock() noexcept
{
  if (mMutexHolder.expired())
    return;

  mMutex.unlock();
}

//==============================================================================
template <typename Iterator>
MultiMutexReference<Iterator>::MultiMutexReference(
    std::weak_ptr<const void> mutexHolder, Iterator first, Iterator last)
  : mMutexHolder(std::move(mutexHolder)), mMutexes(first, last)
{
  mMutexes.reserve(distance(first, last));
  for (; first != last; ++first)
    mMutexes.push_back(ptr(*first));
}

//==============================================================================
template <typename Iterator>
void MultiMutexReference<Iterator>::lock()
{
  if (mMutexHolder.expired())
    return;

  for (auto mutex : mMutexes)
    mutex->lock();
}

//==============================================================================
template <typename Iterator>
bool MultiMutexReference<Iterator>::try_lock() noexcept
{
  if (mMutexHolder.expired())
    return false;

  for (auto mutex : mMutexes)
  {
    if (!mutex->try_lock())
      return false;
  }

  return true;
}

//==============================================================================
template <typename Iterator>
void MultiMutexReference<Iterator>::unlock() noexcept
{
  if (mMutexHolder.expired())
    return;

  for (auto it = mMutexes.rbegin(); it != mMutexes.rend(); ++it)
    (*it)->unlock();
}

//==============================================================================
template <typename Iterator>
template <typename T>
T* MultiMutexReference<Iterator>::ptr(T& obj)
{
  return &obj;
}

//==============================================================================
template <typename Iterator>
template <typename T>
T* MultiMutexReference<Iterator>::ptr(T* obj)
{
  return obj;
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_MUTEXREFERENCE_IMPL_HPP_
