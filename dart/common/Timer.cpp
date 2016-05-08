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

#include "dart/common/Timer.hpp"

#include <ctime>
#include <iostream>
#include <string>

// #if defined _WIN64 || defined _WIN32
// #define CLOCKS_PER_SEC 1000
// #endif

namespace dart {
namespace common {

//==============================================================================
Timer::Timer(const std::string& _name)
  : mCount(0),
    mLastElapsedTime(0.0),
    mTotalElapsedTime(0.0),
    mName(_name),
    mIsStarted(false)
{
#ifdef _WIN32
  mTimer.start.QuadPart = 0;
  mTimer.stop.QuadPart = 0;
  QueryPerformanceFrequency(&mFrequency);
#else
  mStartedTime = 0.0;
  mStoppedTime = 0.0;
#endif
}

//==============================================================================
Timer::~Timer()
{
}

//==============================================================================
#ifdef _WIN32
double Timer::_convLIToSecs(const LARGE_INTEGER& _L)
{
  return (static_cast<double>(_L.QuadPart)
          / static_cast<double>(mFrequency.QuadPart));
}
#endif

//==============================================================================
void Timer::start()
{
  mIsStarted = true;
  mCount++;
#ifdef _WIN32
  QueryPerformanceCounter(&mTimer.start);
#else
  gettimeofday(&mTimeVal, nullptr);
  mStartedTime = mTimeVal.tv_sec + (mTimeVal.tv_usec / 1.0e+6);
#endif
}

//==============================================================================
void Timer::stop()
{
  mIsStarted = false;
#ifdef _WIN32
  QueryPerformanceCounter(&mTimer.stop);
  LARGE_INTEGER time;
  time.QuadPart = mTimer.stop.QuadPart - mTimer.start.QuadPart;
  mLastElapsedTime = _convLIToSecs(time);
#else
  gettimeofday(&mTimeVal, nullptr);
  mStoppedTime = mTimeVal.tv_sec + (mTimeVal.tv_usec / 1.0e+6);
  mLastElapsedTime = mStoppedTime - mStartedTime;
#endif
  mTotalElapsedTime += mLastElapsedTime;
}

//==============================================================================
double Timer::getElapsedTime()
{
#ifdef _WIN32
  LARGE_INTEGER timenow;
  QueryPerformanceCounter(&timenow);
  LARGE_INTEGER time;
  time.QuadPart = timenow.QuadPart - mTimer.start.QuadPart;
  mLastElapsedTime = _convLIToSecs(time);
#else
  gettimeofday(&mTimeVal, nullptr);
  mLastElapsedTime = mTimeVal.tv_sec + (mTimeVal.tv_usec / 1.0e+6)
                     - mStartedTime;
#endif
  return mLastElapsedTime;
}

//==============================================================================
double Timer::getLastElapsedTime() const
{
  return mLastElapsedTime;
}

//==============================================================================
double Timer::getTotalElapsedTime() const
{
  return mTotalElapsedTime;
}

//==============================================================================
bool Timer::isStarted() const
{
  return mIsStarted;
}

//==============================================================================
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
              << "FPS : " << mCount / mTotalElapsedTime << " hz " << std::endl;
  }
  else
  {
    std::cout << "Timer [" << mName << "] doesn't have any record." << std::endl;
  }
}

//==============================================================================
double Timer::getWallTime()
{
#ifdef _WIN32
  LARGE_INTEGER ticksPerSecond;
  LARGE_INTEGER ticks;
  QueryPerformanceFrequency(&ticksPerSecond);
  QueryPerformanceCounter(&ticks);
  return static_cast<double>(ticks.QuadPart)
      / static_cast<double>(ticksPerSecond.QuadPart);
#else
  // Initialize the lastUpdateTime with the current time in seconds
  timeval timeVal;
  gettimeofday(&timeVal, nullptr);
  return timeVal.tv_sec + timeVal.tv_usec / 1.0e+6;
#endif
}

}  // namespace common
}  // namespace dart
