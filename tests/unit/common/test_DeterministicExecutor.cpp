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

#include <dart/common/detail/DeterministicExecutor.h>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

using dart::common::detail::DeterministicExecutor;

TEST(DeterministicExecutor, IndexedChunksSurviveWorkerReconfiguration)
{
  DeterministicExecutor executor;
  const auto submittingThread = std::this_thread::get_id();
  for (const std::size_t workers : {0u, 3u, 1u, 0u, 3u}) {
    executor.setWorkerCount(workers);
    for (const std::size_t count :
         {0u, 1u, 2u, 7u, 127u, 128u, 129u, 255u, 256u, 257u}) {
      SCOPED_TRACE(workers);
      SCOPED_TRACE(count);
      std::array<std::atomic<unsigned>, 257> visits;
      std::array<std::thread::id, 257> threads;
      for (auto& visit : visits)
        visit.store(0u);
      executor.parallelFor(count, 8u, [&](std::size_t index) {
        visits.at(index).fetch_add(1u);
        threads.at(index) = std::this_thread::get_id();
      });
      for (std::size_t index = 0u; index < visits.size(); ++index)
        EXPECT_EQ(index < count ? 1u : 0u, visits[index].load());
      if (count == 0u)
        continue;
      const auto participants = std::min(count, workers + 1u);
      const auto chunk = (count + participants - 1u) / participants;
      EXPECT_EQ(submittingThread, threads[0]);
      for (std::size_t index = 0u; index < count; ++index)
        EXPECT_EQ(threads[(index / chunk) * chunk], threads[index]);
    }
  }
}

TEST(DeterministicExecutor, JoinsAndRethrowsBeforeReusingBorrowedCallable)
{
  DeterministicExecutor executor;
  executor.setWorkerCount(3u);
  for (const bool failMain : {false, true}) {
    std::array<std::atomic<unsigned>, 16> visits;
    for (auto& visit : visits)
      visit.store(0u);
    try {
      executor.parallelFor(16u, 4u, [&](std::size_t index) {
        if (index == 4u)
          throw std::runtime_error("worker");
        if (failMain && index == 0u)
          throw std::runtime_error("main");
        visits[index].fetch_add(1u);
      });
      FAIL() << "The task exception was lost";
    } catch (const std::runtime_error& error) {
      EXPECT_EQ(failMain ? "main" : "worker", std::string(error.what()));
    }
    for (std::size_t index = 8u; index < 16u; ++index)
      EXPECT_EQ(1u, visits[index].load());
    executor.parallelFor(
        16u, 4u, [&](std::size_t index) { visits[index].store(7u); });
    for (const auto& visit : visits)
      EXPECT_EQ(7u, visit.load());
  }
  executor.setWorkerCount(0u);
  EXPECT_THROW(
      executor.parallelFor(
          1u, 1u, [](std::size_t) { throw std::runtime_error("serial"); }),
      std::runtime_error);
  unsigned count = 0u;
  executor.parallelFor(3u, 1u, [&](std::size_t) { ++count; });
  EXPECT_EQ(3u, count);
}

TEST(DeterministicExecutor, RejectsNestedSubmissionAndWorkerReconfiguration)
{
  DeterministicExecutor executor;
  for (const std::size_t workers : {0u, 2u}) {
    executor.setWorkerCount(workers);
    for (const std::size_t nestedIndex : {0u, 3u}) {
      EXPECT_THROW(
          executor.parallelFor(
              8u,
              3u,
              [&](std::size_t index) {
                if (index == nestedIndex)
                  executor.parallelFor(1u, 1u, [](std::size_t) {});
              }),
          std::logic_error);
      EXPECT_THROW(
          executor.parallelFor(
              8u,
              3u,
              [&](std::size_t index) {
                if (index == nestedIndex)
                  executor.setWorkerCount(0u);
              }),
          std::logic_error);
    }
    std::atomic<unsigned> count{0u};
    executor.parallelFor(8u, 3u, [&](std::size_t) { ++count; });
    EXPECT_EQ(8u, count.load());
  }
}

TEST(DeterministicExecutor, IndependentOwnersCanExecuteConcurrently)
{
  DeterministicExecutor first;
  DeterministicExecutor second;
  first.setWorkerCount(2u);
  second.setWorkerCount(2u);
  std::mutex mutex;
  std::condition_variable condition;
  unsigned entered = 0u;
  bool release = false;
  auto run = [&](DeterministicExecutor& executor) {
    std::atomic<unsigned> count{0u};
    executor.parallelFor(12u, 3u, [&](std::size_t index) {
      if (index == 0u) {
        std::unique_lock<std::mutex> lock(mutex);
        ++entered;
        condition.notify_all();
        condition.wait(lock, [&] { return release; });
      }
      ++count;
    });
    return count.load();
  };
  auto firstResult = std::async(std::launch::async, [&] { return run(first); });
  auto secondResult
      = std::async(std::launch::async, [&] { return run(second); });
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(condition.wait_for(
        lock, std::chrono::seconds(5), [&] { return entered == 2u; }));
    release = true;
  }
  condition.notify_all();
  EXPECT_EQ(12u, firstResult.get());
  EXPECT_EQ(12u, secondResult.get());
}
