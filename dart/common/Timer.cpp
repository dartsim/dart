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

#include "dart/common/Timer.h"

#include <ctime>
#include <iostream>
#include <string>

// #if defined _WIN64 || defined _WIN32
// #define CLOCKS_PER_SEC 1000
// #endif

namespace dart {
namespace common {

Timer::Timer(const std::string& _name)
  : mName(_name),
    mCount(0.0),
    mTotalElapsedTime(0.0),
    mIsStarted(false)
{
#if WIN32
  mTimer.start.QuadPart = 0;
  mTimer.stop.QuadPart = 0;
  QueryPerformanceFrequency(&mFrequency);
#endif
}

Timer::~Timer()
{
  print();
}

#if WIN32
double Timer::_convLIToSecs(const LARGE_INTEGER& _L)
{
  return (static_cast<double>(_L.QuadPart)
          / static_cast<double>(mFrequency.QuadPart));
}
#endif

void Timer::start()
{
  mIsStarted = true;
  mCount++;
#if WIN32
  QueryPerformanceCounter(&mTimer.start);
#else
  mStartedTime = clock();
#endif
}

void Timer::stop()
{
  mIsStarted = false;
#if WIN32
  QueryPerformanceCounter(&mTimer.stop);
  LARGE_INTEGER time;
  time.QuadPart = mTimer.stop.QuadPart - mTimer.start.QuadPart;
  mLastElapsedTime = _convLIToSecs(time);
#else
  mStoppedTime = clock();
  mLastElapsedTime = _subtractTimes(mStoppedTime, mStartedTime);
#endif
  mTotalElapsedTime += mLastElapsedTime;
}

double Timer::getElapsedTime()
{
#if WIN32
  LARGE_INTEGER timenow;
  QueryPerformanceCounter(&timenow);
  LARGE_INTEGER time;
  time.QuadPart = timenow.QuadPart - mTimer.start.QuadPart;
  mLastElapsedTime = _convLIToSecs(time);
#else
  double now = clock();
  mLastElapsedTime = _subtractTimes(now, mStartedTime);
#endif
  return mLastElapsedTime;
}

double Timer::getLastElapsedTime() const
{
  return mLastElapsedTime;
}

double Timer::getTotalElapsedTime() const
{
  return mTotalElapsedTime;
}

bool Timer::isStarted() const
{
  return mIsStarted;
}

void Timer::print()
{
  if (mCount > 0)
  {
    std::cout << "Timer [" << mName << "] : " << std::endl
              << "Last elapsed : " << mLastElapsedTime << "; "
              << "Total time : " << " "
              << mTotalElapsedTime << "; "
              << "Total count : " << mCount << "; "
              << "Average time : " << mTotalElapsedTime / mCount << " "
              << "FPS : " << mCount / mTotalElapsedTime << "hz " << std::endl;
  }
  else
  {
    std::cout << "Timer " << mName << " doesn't have any record." << std::endl;
  }
}

double Timer::_subtractTimes(double _endTime, double _startTime)
{
  return (_endTime - _startTime) / CLOCKS_PER_SEC;
}

}  // namespace common
}  // namespace dart
