/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_COMMON_TIMER_H_
#define DART_COMMON_TIMER_H_

#include <string>

#if WIN32
#define NOMINMAX
#include <windows.h>
typedef struct
{
  LARGE_INTEGER  start;
  LARGE_INTEGER  stop;
} stopWatch;
#endif

namespace dart {
namespace common {

/// \brief The implementation of Timer class
///
/// This is a definition of mTimer class.
/// For measure the time, clock() api is used
class Timer
{
public:
  /// \brief Default constructor.
  explicit Timer(const std::string& _name = "Noname Timer");

  /// \brief Default destructor.
  ~Timer();

  /// \brief Start timer.
  void start();

  /// \brief Returns whether the timer is started.
  bool isStarted() const;

  /// \brief Stop the timer.
  void stop();

  /// \brief Return elapsed time in seconds since startTimer().
  /// \see start()
  double getElapsedTime();

  /// \brief
  double getLastElapsedTime() const;

  /// \brief
  double getTotalElapsedTime() const;

  /// \brief
  void print();

private:
  int mCount;

#if WIN32
  stopWatch mTimer;
#else
  double mStartedTime;
  double mStoppedTime;
#endif
  double mLastElapsedTime;
  double mTotalElapsedTime;
  std::string mName;
  bool mIsStarted;

#if WIN32
  LARGE_INTEGER  mFrequency;
  double _convLIToSecs(const LARGE_INTEGER& _L);
#endif

  double _subtractTimes(double _endTime, double _startTime);
};

}  // namespace common
}  // namespace dart

#endif  // DART_COMMON_TIMER_H_
