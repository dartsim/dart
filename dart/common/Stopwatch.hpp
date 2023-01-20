/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#pragma once

#include <dart/common/Fwd.hpp>

#include <chrono>
#include <iostream>

namespace dart::common {

/// Stopwatch class for measuring elapsed time.
///
/// Stopwatch is a simple class for measuring elapsed time. It is a wrapper of
/// std::chrono::high_resolution_clock. The default unit of the elapsed time is
/// std::chrono::seconds.
///
/// \tparam UnitType: The unit type of the elapsed time. The default is
/// std::chrono::nanoseconds.
/// \tparam ClockType: The clock type of the stopwatch. The default is
/// std::chrono::high_resolution_clock.
template <
    typename UnitType = std::chrono::nanoseconds,
    typename ClockType = std::chrono::high_resolution_clock>
class Stopwatch final
{
public:
  /// Constructor
  ///
  /// \param[in] start: (optional) Whether to start the stopwatch on
  /// construction
  explicit Stopwatch(bool start = true);

  /// Destructor
  ~Stopwatch();

  /// Returns whether the stopwatch is started.
  [[nodiscard]] bool isStarted() const;

  /// Starts the stopwatch
  void start();

  /// Stops the stopwatch
  void stop();

  /// Resets the stopwatch.
  ///
  /// Sets the start time to current time and the accumulated elasped time to
  /// zero.
  void reset();

  /// Returns the elapsed time in seconds.
  [[nodiscard]] double elapsedS() const;

  /// Returns the elapsed time in milliseconds.
  [[nodiscard]] double elapsedMS() const;

  /// Returns the elapsed time in microseconds.
  [[nodiscard]] double elapsedUS() const;

  /// Returns the elapsed time in nanoseconds.
  [[nodiscard]] double elapsedNS() const;

  /// Prints state of the stopwatch
  void print(std::ostream& os = std::cout) const;

  /// Prints state of the stopwatch
  template <typename T, typename U>
  friend std::ostream& operator<<(std::ostream& os, const Stopwatch<T, U>& sw);

private:
  // Deletes copy/move constructors and assign/move operators
  Stopwatch(const Stopwatch&) = delete;
  Stopwatch(Stopwatch&&) = delete;
  Stopwatch& operator=(const Stopwatch&) = delete;
  Stopwatch& operator=(Stopwatch&&) = delete;

  /// Returns the duration.
  [[nodiscard]] UnitType duration() const;

  /// The start time
  typename ClockType::time_point mStart;

  /// The elapsed time
  UnitType mElapsed;

  /// Whether the stopwatch is paused.
  bool mPaused;
};

/// MATLAB like timer
///
/// \code
/// tic();
/// auto elapsedS = toc();  // prints and returns the elapsed time
/// \endcode
DART_COMMON_API void tic();

/// Returns the elapsed time in seconds since the last tic() call.
DART_COMMON_API double toc(bool print = false);

/// Returns the elapsed time in seconds since the last tic() call.
DART_COMMON_API double tocS(bool print = false);

/// Returns the elapsed time in milliseconds since the last tic() call.
DART_COMMON_API double tocMS(bool print = false);

/// Returns the elapsed time in microseconds since the last tic() call.
DART_COMMON_API double tocUS(bool print = false);

/// Returns the elapsed time in nanoseconds since the last tic() call.
DART_COMMON_API double tocNS(bool print = false);

/// Stopwatch with std::chrono::seconds as the unit type
using StopwatchS = Stopwatch<std::chrono::seconds>;

/// Stopwatch with std::chrono::milliseconds as the unit type
using StopwatchMS = Stopwatch<std::chrono::milliseconds>;

/// Stopwatch with std::chrono::microseconds as the unit type
using StopwatchUS = Stopwatch<std::chrono::microseconds>;

/// Stopwatch with std::chrono::nanoseconds as the unit type
using StopwatchNS = Stopwatch<std::chrono::nanoseconds>;

} // namespace dart::common

#include <dart/common/detail/Stopwatch-impl.hpp>
