/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_COMMON_TIMER_HPP_
#define DART_COMMON_TIMER_HPP_

#include <string>

#ifdef _WIN32
  #ifdef NOMINMAX
    #include <windows.h>
  #else
    #define NOMINMAX
    #include <windows.h>
    #undef NOMINMAX
  #endif
  typedef struct
  {
    LARGE_INTEGER  start;
    LARGE_INTEGER  stop;
  } stopWatch;
#else
  #include <sys/time.h>
#endif

namespace dart {
namespace common {

/// \brief The implementation of Timer class
///
/// This is a definition of mTimer class.
/// For measure the time, gettimeofday() api is used
class Timer
{
public:
  /// \brief Default constructor
  explicit Timer(const std::string& _name = "Noname Timer");

  /// \brief Default destructor
  virtual ~Timer();

  /// \brief Start timer
  void start();

  /// \brief Returns whether the timer is started
  bool isStarted() const;

  /// \brief Stop the timer
  void stop();

  /// \brief Return elapsed time in seconds since startTimer()
  /// \see start()
  double getElapsedTime();

  /// \brief Return last elapsed time in seconds
  double getLastElapsedTime() const;

  /// \brief Return total elapsed time in seconds
  double getTotalElapsedTime() const;

  /// \brief Print results
  void print();

  /// \brief Return the current time of the system in seconds
  static double getWallTime();

private:
  int mCount;

#ifdef _WIN32
  stopWatch mTimer;
#else
  timeval mTimeVal;
  double mStartedTime;
  double mStoppedTime;
#endif

  double mLastElapsedTime;
  double mTotalElapsedTime;
  std::string mName;
  bool mIsStarted;

#ifdef _WIN32
  LARGE_INTEGER  mFrequency;
  double _convLIToSecs(const LARGE_INTEGER& _L);
#endif
};

}  // namespace common
}  // namespace dart

#endif  // DART_COMMON_TIMER_HPP_
