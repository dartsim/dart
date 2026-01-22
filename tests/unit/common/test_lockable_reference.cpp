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

#include <dart/common/lockable_reference.hpp>

#include <gtest/gtest.h>

#include <mutex>
#include <vector>

using namespace dart;
using namespace dart::common;

//==============================================================================
TEST(SingleLockableReference, LockAndUnlock)
{
  auto holder = std::make_shared<int>(42);
  std::mutex mutex;

  SingleLockableReference<std::mutex> lockRef(holder, mutex);

  lockRef.lock();
  lockRef.unlock();
}

TEST(SingleLockableReference, TryLockSucceedsWhenUnlocked)
{
  auto holder = std::make_shared<int>(42);
  std::mutex mutex;

  SingleLockableReference<std::mutex> lockRef(holder, mutex);

  EXPECT_TRUE(lockRef.try_lock());
  lockRef.unlock();
}

TEST(SingleLockableReference, TryLockFailsWhenLocked)
{
  auto holder = std::make_shared<int>(42);
  std::mutex mutex;

  mutex.lock();
  SingleLockableReference<std::mutex> lockRef(holder, mutex);

  EXPECT_FALSE(lockRef.try_lock());
  mutex.unlock();
}

TEST(SingleLockableReference, WorksWithStdLockGuard)
{
  auto holder = std::make_shared<int>(42);
  std::mutex mutex;

  SingleLockableReference<std::mutex> lockRef(holder, mutex);

  {
    std::lock_guard<SingleLockableReference<std::mutex>> guard(lockRef);
    EXPECT_FALSE(mutex.try_lock());
  }

  EXPECT_TRUE(mutex.try_lock());
  mutex.unlock();
}

TEST(SingleLockableReference, LockNoOpWhenHolderExpired)
{
  std::mutex mutex;
  SingleLockableReference<std::mutex>* lockRef = nullptr;

  {
    auto holder = std::make_shared<int>(42);
    lockRef = new SingleLockableReference<std::mutex>(holder, mutex);
  }

  lockRef->lock();
  EXPECT_TRUE(mutex.try_lock());
  mutex.unlock();

  delete lockRef;
}

TEST(SingleLockableReference, TryLockReturnsFalseWhenHolderExpired)
{
  std::mutex mutex;
  SingleLockableReference<std::mutex>* lockRef = nullptr;

  {
    auto holder = std::make_shared<int>(42);
    lockRef = new SingleLockableReference<std::mutex>(holder, mutex);
  }

  EXPECT_FALSE(lockRef->try_lock());

  delete lockRef;
}

TEST(SingleLockableReference, UnlockNoOpWhenHolderExpired)
{
  std::mutex mutex;
  SingleLockableReference<std::mutex>* lockRef = nullptr;

  {
    auto holder = std::make_shared<int>(42);
    lockRef = new SingleLockableReference<std::mutex>(holder, mutex);
    lockRef->lock();
  }

  lockRef->unlock();

  delete lockRef;
}

//==============================================================================
TEST(MultiLockableReference, LockAndUnlockMultipleMutexes)
{
  auto holder = std::make_shared<int>(42);
  std::vector<std::mutex> mutexes(3);
  std::vector<std::mutex*> mutexPtrs;
  for (auto& m : mutexes) {
    mutexPtrs.push_back(&m);
  }

  MultiLockableReference<std::mutex> lockRef(
      holder, mutexPtrs.begin(), mutexPtrs.end());

  lockRef.lock();

  for (auto& m : mutexes) {
    EXPECT_FALSE(m.try_lock());
  }

  lockRef.unlock();

  for (auto& m : mutexes) {
    EXPECT_TRUE(m.try_lock());
    m.unlock();
  }
}

TEST(MultiLockableReference, TryLockSucceedsWhenAllUnlocked)
{
  auto holder = std::make_shared<int>(42);
  std::vector<std::mutex> mutexes(2);
  std::vector<std::mutex*> mutexPtrs;
  for (auto& m : mutexes) {
    mutexPtrs.push_back(&m);
  }

  MultiLockableReference<std::mutex> lockRef(
      holder, mutexPtrs.begin(), mutexPtrs.end());

  EXPECT_TRUE(lockRef.try_lock());
  lockRef.unlock();
}

TEST(MultiLockableReference, TryLockFailsWhenOneLocked)
{
  auto holder = std::make_shared<int>(42);
  std::vector<std::mutex> mutexes(3);
  std::vector<std::mutex*> mutexPtrs;
  for (auto& m : mutexes) {
    mutexPtrs.push_back(&m);
  }

  mutexes[1].lock();

  MultiLockableReference<std::mutex> lockRef(
      holder, mutexPtrs.begin(), mutexPtrs.end());

  EXPECT_FALSE(lockRef.try_lock());

  mutexes[1].unlock();
}

TEST(MultiLockableReference, WorksWithStdLockGuard)
{
  auto holder = std::make_shared<int>(42);
  std::vector<std::mutex> mutexes(2);
  std::vector<std::mutex*> mutexPtrs;
  for (auto& m : mutexes) {
    mutexPtrs.push_back(&m);
  }

  MultiLockableReference<std::mutex> lockRef(
      holder, mutexPtrs.begin(), mutexPtrs.end());

  {
    std::lock_guard<MultiLockableReference<std::mutex>> guard(lockRef);
    for (auto& m : mutexes) {
      EXPECT_FALSE(m.try_lock());
    }
  }

  for (auto& m : mutexes) {
    EXPECT_TRUE(m.try_lock());
    m.unlock();
  }
}

TEST(MultiLockableReference, LockNoOpWhenHolderExpired)
{
  std::vector<std::mutex> mutexes(2);
  std::vector<std::mutex*> mutexPtrs;
  for (auto& m : mutexes) {
    mutexPtrs.push_back(&m);
  }

  MultiLockableReference<std::mutex>* lockRef = nullptr;

  {
    auto holder = std::make_shared<int>(42);
    lockRef = new MultiLockableReference<std::mutex>(
        holder, mutexPtrs.begin(), mutexPtrs.end());
  }

  lockRef->lock();

  for (auto& m : mutexes) {
    EXPECT_TRUE(m.try_lock());
    m.unlock();
  }

  delete lockRef;
}

TEST(MultiLockableReference, TryLockReturnsFalseWhenHolderExpired)
{
  std::vector<std::mutex> mutexes(2);
  std::vector<std::mutex*> mutexPtrs;
  for (auto& m : mutexes) {
    mutexPtrs.push_back(&m);
  }

  MultiLockableReference<std::mutex>* lockRef = nullptr;

  {
    auto holder = std::make_shared<int>(42);
    lockRef = new MultiLockableReference<std::mutex>(
        holder, mutexPtrs.begin(), mutexPtrs.end());
  }

  EXPECT_FALSE(lockRef->try_lock());

  delete lockRef;
}

TEST(MultiLockableReference, SingleMutexBehavesLikeSingle)
{
  auto holder = std::make_shared<int>(42);
  std::mutex mutex;
  std::vector<std::mutex*> mutexPtrs{&mutex};

  MultiLockableReference<std::mutex> lockRef(
      holder, mutexPtrs.begin(), mutexPtrs.end());

  lockRef.lock();
  EXPECT_FALSE(mutex.try_lock());
  lockRef.unlock();
  EXPECT_TRUE(mutex.try_lock());
  mutex.unlock();
}
