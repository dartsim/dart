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

#include "dart/common/Logging.hpp"

#include <atomic>

namespace dart::common {

namespace {
// One immutable {callback, userData} pair per setLogCallback() call, published
// through a single lock-free pointer, so a reader never pairs one call's
// callback with another call's user data. Every slot stays reachable through
// \c previous: an in-flight forwardToLogCallback() may still be reading an old
// one, so none is freed, and the chain keeps leak checkers quiet. The cost is
// one small allocation per setLogCallback() call.
struct Slot
{
  LogCallback callback;
  void* userData;
  const Slot* previous;
};

std::atomic<const Slot*> sSlot{nullptr};
static_assert(
    std::atomic<const Slot*>::is_always_lock_free,
    "the logging path must not take a lock, so that fork() cannot inherit it "
    "held");

// Set while this thread runs the callback: a log call from inside it goes to
// the default output rather than back into the callback. The child of a
// fork() inherits the forking thread's copy, so do not fork from inside the
// callback.
thread_local bool tInsideCallback = false;

// Set by the first callback that throws, so that the throw is reported once.
// Namespace scope, so no guard lock sits on the throw-report path.
std::atomic<bool> sThrowReported{false};

struct InsideCallback
{
  InsideCallback()
  {
    tInsideCallback = true;
  }
  ~InsideCallback()
  {
    tInsideCallback = false;
  }
};
} // namespace

void setLogCallback(LogCallback callback, void* userData)
{
  const Slot* previous = sSlot.load(std::memory_order_acquire);
  sSlot.store(
      new Slot{callback, callback ? userData : nullptr, previous},
      std::memory_order_release);
}

namespace detail {

bool hasLogCallback()
{
  const Slot* slot = sSlot.load(std::memory_order_acquire);
  return slot && slot->callback;
}

bool forwardToLogCallback(
    LogLevel level, const char* message, std::size_t length)
{
  const Slot* slot = sSlot.load(std::memory_order_acquire);
  if (!slot || !slot->callback || tInsideCallback)
    return false;
  InsideCallback guard;
  try {
    slot->callback(level, message, length, slot->userData);
    return true;
  } catch (...) {
    if (!sThrowReported.exchange(true)) {
      // Inside the guard, so this reaches spdlog or the console, not the
      // callback.
      error(
          "A log callback threw. DART ignores the exception and sends the "
          "message to the default output; later throws are not reported.");
    }
    return false;
  }
}

} // namespace detail

} // namespace dart::common
