/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <chrono>
#include <iostream>
#include <ostream>

#include "dart/common/export.hpp"

namespace dart::common {

template <
    typename UnitType,
    typename ClockType = std::chrono::high_resolution_clock>
class Stopwatch final
{
public:
  explicit Stopwatch(bool start = true);

  ~Stopwatch();

  bool is_started() const;

  void start();

  void stop();

  void reset();

  double elapsed_s() const;

  double elapsed_ms() const;

  double elapsed_us() const;

  double elapsed_ns() const;

  void print(std::ostream& os = std::cout) const;

  template <typename T, typename U>
  friend std::ostream& operator<<(std::ostream& os, const Stopwatch<T, U>& sw);

private:
  Stopwatch(const Stopwatch&) = delete;
  Stopwatch(Stopwatch&&) = delete;
  Stopwatch& operator=(const Stopwatch&) = delete;
  Stopwatch& operator=(Stopwatch&&) = delete;

  UnitType duration() const;

  typename ClockType::time_point m_start;
  UnitType m_elapsed;
  bool m_paused;
};

DART_COMMON_API void tic();

DART_COMMON_API double toc(bool print = false);
DART_COMMON_API double toc_s(bool print = false);
DART_COMMON_API double toc_ms(bool print = false);
DART_COMMON_API double toc_us(bool print = false);
DART_COMMON_API double toc_ns(bool print = false);

using StopwatchS = Stopwatch<std::chrono::seconds>;
using StopwatchMS = Stopwatch<std::chrono::milliseconds>;
using StopwatchUS = Stopwatch<std::chrono::microseconds>;
using StopwatchNS = Stopwatch<std::chrono::nanoseconds>;

using Timer = StopwatchNS;

} // namespace dart::common

#include "dart/common/detail/stopwatch_impl.hpp"
