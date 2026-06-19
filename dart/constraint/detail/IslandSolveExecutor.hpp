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

#ifndef DART_CONSTRAINT_DETAIL_ISLANDSOLVEEXECUTOR_HPP_
#define DART_CONSTRAINT_DETAIL_ISLANDSOLVEEXECUTOR_HPP_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include <cstddef>

namespace dart {
namespace constraint {
namespace detail {

/// A tiny persistent worker pool for running an index-parallel loop. World owns
/// the pool and shares it across independent per-skeleton loops and constraint
/// island solving, so per-step thread spawning (and the loss of thread_local
/// LCP scratch) is avoided.
///
/// Work items are pulled by an atomic counter (dynamic load balancing), and the
/// calling thread participates as one of the workers. The body is run for every
/// index in [0, count); because the constraint-island work items are
/// independent and order-agnostic, the result is identical regardless of how
/// indices are distributed across threads or how many threads there are.
class IslandSolveExecutor
{
public:
  /// Creates a pool that runs work on \c numThreads threads total (the caller
  /// plus numThreads - 1 persistent workers). numThreads is clamped to >= 1.
  explicit IslandSolveExecutor(std::size_t numThreads)
    : mNumThreads(numThreads < 1 ? 1 : numThreads)
  {
    try {
      for (std::size_t i = 1; i < mNumThreads; ++i)
        mWorkers.emplace_back([this] { workerLoop(); });
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(mMutex);
        mStop = true;
      }
      mStartCv.notify_all();
      for (std::thread& worker : mWorkers) {
        if (worker.joinable())
          worker.join();
      }
      throw;
    }
  }

  ~IslandSolveExecutor()
  {
    {
      std::lock_guard<std::mutex> lock(mMutex);
      mStop = true;
    }
    mStartCv.notify_all();
    for (std::thread& worker : mWorkers) {
      if (worker.joinable())
        worker.join();
    }
  }

  IslandSolveExecutor(const IslandSolveExecutor&) = delete;
  IslandSolveExecutor& operator=(const IslandSolveExecutor&) = delete;

  std::size_t getNumThreads() const
  {
    return mNumThreads;
  }

  /// Runs body(i) for every i in [0, count), distributing the indices across
  /// the worker pool and the calling thread, and blocks until all are done. Any
  /// exception thrown by a body is captured and rethrown on the calling thread
  /// after the join.
  void run(std::size_t count, const std::function<void(std::size_t)>& body)
  {
    if (count == 0)
      return;

    if (mWorkers.empty()) {
      // Degenerate single-thread pool: just run inline.
      for (std::size_t i = 0; i < count; ++i)
        body(i);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mMutex);
      mBody = &body;
      mCount = count;
      mNext.store(0, std::memory_order_relaxed);
      mPending = mWorkers.size();
      mError = nullptr;
      ++mGeneration;
    }
    mStartCv.notify_all();

    // The calling thread participates as a worker too.
    runWorkItems();

    std::unique_lock<std::mutex> lock(mMutex);
    mDoneCv.wait(lock, [this] { return mPending == 0; });
    mBody = nullptr;
    std::exception_ptr error = mError;
    mError = nullptr;
    lock.unlock();

    if (error)
      std::rethrow_exception(error);
  }

private:
  void runWorkItems()
  {
    for (;;) {
      const std::size_t i = mNext.fetch_add(1, std::memory_order_relaxed);
      if (i >= mCount)
        break;
      try {
        (*mBody)(i);
      } catch (...) {
        std::lock_guard<std::mutex> lock(mMutex);
        if (!mError)
          mError = std::current_exception();
        // Stop this worker from grabbing further items in this generation.
        mNext.store(mCount, std::memory_order_relaxed);
        break;
      }
    }
  }

  void workerLoop()
  {
    std::size_t lastGeneration = 0;
    for (;;) {
      std::unique_lock<std::mutex> lock(mMutex);
      mStartCv.wait(lock, [this, lastGeneration] {
        return mStop || mGeneration != lastGeneration;
      });
      if (mStop)
        return;
      lastGeneration = mGeneration;
      lock.unlock();

      runWorkItems();

      lock.lock();
      if (--mPending == 0)
        mDoneCv.notify_one();
    }
  }

  const std::size_t mNumThreads;
  std::vector<std::thread> mWorkers;

  std::mutex mMutex;
  std::condition_variable mStartCv;
  std::condition_variable mDoneCv;

  // Current job (protected by mMutex except mNext which is atomic).
  const std::function<void(std::size_t)>* mBody = nullptr;
  std::size_t mCount = 0;
  std::atomic<std::size_t> mNext{0};
  std::size_t mPending = 0;
  std::size_t mGeneration = 0;
  bool mStop = false;
  std::exception_ptr mError;
};

} // namespace detail
} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_DETAIL_ISLANDSOLVEEXECUTOR_HPP_
