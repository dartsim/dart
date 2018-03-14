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

#include "dart/common/MutexReference.hpp"

namespace dart {
namespace common {

//==============================================================================
SingleMutexReference::SingleMutexReference(
    std::weak_ptr<const void> mutexHolder, std::mutex& mutex) noexcept
    : mMutexHolder(std::move(mutexHolder)),
      mMutex(mutex)
{
  // Do nothing
}

//==============================================================================
void SingleMutexReference::lock()
{
  if (mMutexHolder.expired())
    return;

  mMutex.lock();
}

//==============================================================================
bool SingleMutexReference::try_lock() noexcept
{
  if (mMutexHolder.expired())
    return false;

  return mMutex.try_lock();
}

//==============================================================================
void SingleMutexReference::unlock() noexcept
{
  if (mMutexHolder.expired())
    return;

  mMutex.unlock();
}

//==============================================================================
MultiMutexReference::MultiMutexReference(
    std::weak_ptr<const void> mutexHolder,
    const std::set<std::mutex*>& mutexes) noexcept
    : mMutexHolder(std::move(mutexHolder)),
      mMutexes(mutexes)
{
  // Do nothing
}

//==============================================================================
void MultiMutexReference::lock()
{
  if (mMutexHolder.expired())
    return;

  for (auto& mutex : mMutexes)
    mutex->lock();
}

//==============================================================================
bool MultiMutexReference::try_lock() noexcept
{
  if (mMutexHolder.expired())
    return false;

  for (auto& mutex : mMutexes)
  {
    if (!mutex->try_lock())
      return false;
  }

  return true;
}

//==============================================================================
void MultiMutexReference::unlock() noexcept
{
  if (mMutexHolder.expired())
    return;

  for (auto it = mMutexes.rbegin(); it != mMutexes.rend(); ++it)
    (*it)->unlock();
}

} // namespace common
} // namespace dart
