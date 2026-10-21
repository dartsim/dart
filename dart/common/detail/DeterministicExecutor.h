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

#ifndef DART_COMMON_DETAIL_DETERMINISTICEXECUTOR_H_
#define DART_COMMON_DETAIL_DETERMINISTICEXECUTOR_H_

#include <algorithm>
#include <condition_variable>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>

#include <cstddef>

namespace dart {
namespace common {
namespace detail {

// Fixed-chunk parallel loop behind the native simulation pools. Submissions
// to one executor run one at a time, serial ones included, and a task's
// exception is rethrown after every participant finishes, lowest index first.
class DeterministicExecutor
{
public:
  DeterministicExecutor() = default;

  ~DeterministicExecutor()
  {
    stopWorkers();
  }

  void setWorkerCount(std::size_t workerCount)
  {
    requireExternalSubmission();
    std::lock_guard<std::mutex> submitLock(mSubmitMutex);

    if (workerCount == mWorkers.size())
      return;

    stopWorkers();
    if (workerCount == 0u)
      return;

    {
      std::lock_guard<std::mutex> lock(mMutex);
      mStop = false;
    }

    try {
      mWorkers.reserve(workerCount);
      for (std::size_t i = 0u; i < workerCount; ++i)
        mWorkers.emplace_back([this] { workerLoop(); });
    } catch (...) {
      stopWorkers();
      throw;
    }
  }

  template <typename Func>
  void parallelFor(std::size_t count, std::size_t numThreads, Func&& func)
  {
    if (count == 0u)
      return;

    requireExternalSubmission();
    std::lock_guard<std::mutex> submitLock(mSubmitMutex);

    using Function = typename std::remove_reference<Func>::type;
    const TaskInvoker invoker
        = [](void* callable, std::size_t begin, std::size_t end) {
            auto& task = *static_cast<Function*>(callable);
            for (std::size_t i = begin; i < end; ++i)
              task(i);
          };
    void* callable = static_cast<void*>(std::addressof(func));
    const std::size_t totalParticipants = std::min<std::size_t>(
        std::min<std::size_t>(numThreads, count), mWorkers.size() + 1u);
    if (totalParticipants <= 1u) {
      if (const auto exception = invokeRange(callable, invoker, 0u, count))
        std::rethrow_exception(exception);
      return;
    }

    const std::size_t chunkSize
        = (count + totalParticipants - 1u) / totalParticipants;
    const std::size_t workerCount = totalParticipants - 1u;

    {
      std::lock_guard<std::mutex> lock(mMutex);
      mTaskActive = true;
      mTaskCallable = callable;
      mTaskInvoker = invoker;
      mTaskCount = count;
      mTaskChunkSize = chunkSize;
      mWorkerLimit = totalParticipants;
      mNextWorkerIndex = 1u;
      mActiveWorkerCount = workerCount;
      mException = nullptr;
      mExceptionParticipant = totalParticipants;
      ++mTaskGeneration;
    }

    mTaskCv.notify_all();

    const std::size_t mainEnd = std::min<std::size_t>(count, chunkSize);
    const auto mainException
        = invokeRange(mTaskCallable, mTaskInvoker, 0u, mainEnd);

    // Join every participant before a borrowed callable can unwind.
    std::exception_ptr exception;
    {
      std::unique_lock<std::mutex> lock(mMutex);
      if (mainException) {
        mException = mainException;
        mExceptionParticipant = 0u;
      }
      mDoneCv.wait(lock, [this] { return mActiveWorkerCount == 0u; });
      exception = mException;
      mException = nullptr;
      mTaskActive = false;
      mTaskCallable = nullptr;
      mTaskInvoker = nullptr;
      mWorkerLimit = 1u;
    }
    if (exception)
      std::rethrow_exception(exception);
  }

private:
  using TaskInvoker = void (*)(void*, std::size_t, std::size_t);

  struct ActiveFrame
  {
    const DeterministicExecutor* executor;
    const ActiveFrame* previous;
  };

  void requireExternalSubmission() const
  {
    // ponytail: sees only this thread's tasks; a cycle through another
    // executor's worker thread still deadlocks. Extend for shared execution.
    for (auto* frame = mActiveFrame; frame; frame = frame->previous) {
      if (frame->executor == this)
        throw std::logic_error("Nested submission to the same executor");
    }
  }

  std::exception_ptr invokeRange(
      void* callable,
      TaskInvoker invoker,
      std::size_t begin,
      std::size_t end) noexcept
  {
    const ActiveFrame frame{this, mActiveFrame};
    mActiveFrame = &frame;
    std::exception_ptr exception;
    try {
      invoker(callable, begin, end);
    } catch (...) {
      exception = std::current_exception();
    }
    mActiveFrame = frame.previous;
    return exception;
  }

  void stopWorkers()
  {
    {
      std::lock_guard<std::mutex> lock(mMutex);
      mStop = true;
      ++mTaskGeneration;
    }

    mTaskCv.notify_all();

    for (auto& worker : mWorkers) {
      if (worker.joinable())
        worker.join();
    }
    mWorkers.clear();

    std::lock_guard<std::mutex> lock(mMutex);
    mStop = false;
    mTaskActive = false;
    mTaskCallable = nullptr;
    mTaskInvoker = nullptr;
    mActiveWorkerCount = 0u;
    mWorkerLimit = 1u;
    mNextWorkerIndex = 1u;
  }

  void workerLoop()
  {
    std::size_t observedGeneration = 0u;

    while (true) {
      void* callable = nullptr;
      TaskInvoker invoker = nullptr;
      std::size_t begin = 0u;
      std::size_t end = 0u;
      std::size_t workerIndex = 0u;

      {
        std::unique_lock<std::mutex> lock(mMutex);
        mTaskCv.wait(lock, [&] {
          return mStop || observedGeneration != mTaskGeneration;
        });

        if (mStop)
          return;

        observedGeneration = mTaskGeneration;
        if (!mTaskActive || mNextWorkerIndex >= mWorkerLimit)
          continue;

        workerIndex = mNextWorkerIndex++;
        begin = workerIndex * mTaskChunkSize;
        end = std::min<std::size_t>(mTaskCount, begin + mTaskChunkSize);
        callable = mTaskCallable;
        invoker = mTaskInvoker;
      }

      const auto exception = invokeRange(callable, invoker, begin, end);

      {
        std::lock_guard<std::mutex> lock(mMutex);
        if (exception && workerIndex < mExceptionParticipant) {
          mException = exception;
          mExceptionParticipant = workerIndex;
        }
        if (mActiveWorkerCount > 0u)
          --mActiveWorkerCount;
        if (mActiveWorkerCount == 0u)
          mDoneCv.notify_one();
      }
    }
  }

  std::vector<std::thread> mWorkers;
  std::mutex mSubmitMutex;
  std::mutex mMutex;
  std::condition_variable mTaskCv;
  std::condition_variable mDoneCv;
  bool mStop{false};
  bool mTaskActive{false};
  std::size_t mTaskGeneration{0u};
  std::size_t mTaskCount{0u};
  std::size_t mTaskChunkSize{0u};
  std::size_t mWorkerLimit{1u};
  std::size_t mNextWorkerIndex{1u};
  std::size_t mActiveWorkerCount{0u};
  void* mTaskCallable{nullptr};
  TaskInvoker mTaskInvoker{nullptr};
  std::exception_ptr mException;
  std::size_t mExceptionParticipant{0u};
  inline static thread_local const ActiveFrame* mActiveFrame = nullptr;
};

} // namespace detail
} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_DETERMINISTICEXECUTOR_H_
