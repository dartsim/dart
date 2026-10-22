// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include <dart/common/Logging.hpp>

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace dart::python {
namespace {

struct LogQueue
{
  static constexpr std::size_t capacity = 100000;
  std::mutex mutex;
  std::deque<std::pair<common::LogLevel, std::string>> records;
  std::atomic<std::size_t> dropped{0};
};

struct BridgeSink
{
  std::shared_ptr<common::LogSink> previous;
  std::atomic<bool> enabled{true};
};

// No Python objects are owned by the native sink, including at shutdown.
struct LoggingBridge
{
  std::shared_ptr<LogQueue> queue = std::make_shared<LogQueue>();
  std::shared_ptr<BridgeSink> sink;
  std::shared_ptr<common::LogSink> installedSink;
  common::LogLevel previousLevel = common::LogLevel::Info;
  std::mutex flushMutex;
  bool exitRegistered = false;

  ~LoggingBridge()
  {
    try {
      disable();
    } catch (...) {
      // Module teardown must not throw or touch Python.
    }
  }

  void enable(int level)
  {
    if (!sink || !sink->enabled) {
      previousLevel = common::getLogLevel();
      auto state = std::make_shared<BridgeSink>();
      installedSink = common::detail::wrapLogSink(
          [state, queue = queue](std::shared_ptr<common::LogSink> previous) {
            state->previous = std::move(previous);
            return [state, queue](
                       common::LogLevel severity, const std::string& text) {
              if (!state->enabled) {
                if (state->previous)
                  (*state->previous)(severity, text);
                else
                  common::detail::defaultLogSink(severity, text);
                return;
              }
              std::lock_guard<std::mutex> lock(queue->mutex);
              queue->records.emplace_back(severity, text);
              if (queue->records.size() > LogQueue::capacity) {
                queue->records.pop_front();
                queue->dropped.fetch_add(1, std::memory_order_relaxed);
              }
            };
          });
      sink = std::move(state);
    }
    const auto threshold = level <= 10   ? common::LogLevel::Trace
                           : level <= 20 ? common::LogLevel::Info
                           : level <= 30 ? common::LogLevel::Warn
                           : level <= 40 ? common::LogLevel::Error
                           : level <= 50 ? common::LogLevel::Fatal
                                         : common::LogLevel::Off;
    common::setLogLevel(threshold);
  }

  void disable()
  {
    if (!sink || !sink->enabled.exchange(false))
      return;
    common::detail::restoreLogSinkIf(installedSink, sink->previous);
    common::setLogLevel(previousLevel);
  }

  std::size_t flush()
  {
    static thread_local bool draining = false;
    if (draining)
      return 0;
    std::unique_lock<std::mutex> lock(flushMutex, std::defer_lock);
    {
      // A handler may release the GIL; another drainer must wait without it.
      nb::gil_scoped_release release;
      lock.lock();
    }
    draining = true;
    std::size_t count = 0;
    try {
      const auto logger
          = nb::module_::import_("logging").attr("getLogger")("dartpy");
      std::size_t pending;
      std::size_t dropped;
      {
        std::lock_guard<std::mutex> lock(queue->mutex);
        pending = queue->records.size();
        dropped = queue->dropped.exchange(0, std::memory_order_relaxed);
      }
      if (dropped)
        logger.attr("log")(
            30, "dartpy: " + std::to_string(dropped) + " log records dropped");
      // Drain one snapshot: handler-generated records wait for the next flush.
      static constexpr int levels[]{10, 10, 20, 30, 40, 50};
      // Disabling in a handler drains the rest as this outer flush unwinds.
      for (; count < pending || !sink || !sink->enabled; ++count) {
        std::pair<common::LogLevel, std::string> record;
        {
          std::lock_guard<std::mutex> lock(queue->mutex);
          if (queue->records.empty())
            break;
          record = std::move(queue->records.front());
          queue->records.pop_front();
        }
        const auto index = static_cast<int>(record.first);
        logger.attr("log")(
            index >= 0 && index < 6 ? levels[index] : 50, record.second);
      }
    } catch (...) {
      draining = false;
      throw;
    }
    draining = false;
    return count;
  }
};

} // namespace

void Logging(nb::module_& m)
{
  auto bridge = std::make_shared<LoggingBridge>();
  m.def(
      "enable_logging",
      [bridge](int level) {
        if (!bridge->exitRegistered) {
          // Register after logging.shutdown so queued records drain first.
          nb::module_::import_("logging");
          nb::module_::import_("atexit").attr("register")(
              nb::module_::import_("dartpy").attr("common").attr(
                  "disable_logging"));
          bridge->exitRegistered = true;
        }
        bridge->enable(level);
      },
      nb::kw_only(),
      nb::arg("level") = 20,
      "Queue DART diagnostics for Python logging; level uses logging "
      "constants. Retain at most 100,000 records, dropping the oldest on "
      "overflow and warning at the next flush.");
  m.def(
      "flush_logs",
      [bridge] { return bridge->flush(); },
      "Deliver queued records to logging.getLogger('dartpy') under the GIL. "
      "Same-thread reentry returns zero; other threads wait. Handlers must "
      "not join a thread that calls flush_logs(). Drop counts reset before "
      "the overflow warning is delivered, even if a handler raises.");
  m.def(
      "disable_logging",
      [bridge] {
        bridge->disable();
        return bridge->flush();
      },
      "Disable queueing and restore the previous native sink if still "
      "installed, restore the threshold, then drain pending records. In a "
      "handler, the outer flush drains the remaining queue.");
  m.def("trace", [](const std::string& log) { common::trace("{}", log); });
  m.def("debug", [](const std::string& log) { common::debug("{}", log); });
  m.def("info", [](const std::string& log) { common::info("{}", log); });
  m.def("warn", [](const std::string& log) { common::warn("{}", log); });
  m.def("error", [](const std::string& log) { common::error("{}", log); });
  m.def("fatal", [](const std::string& log) { common::fatal("{}", log); });
}

} // namespace dart::python
