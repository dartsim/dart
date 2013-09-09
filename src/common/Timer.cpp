/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/12/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include <iostream>
#include <string.h>
#include <time.h>
#include "Timer.h"  // for WIN32 placing this before glog/logging.h causes compilation errors

//#if defined _WIN64 || defined _WIN32
//#define CLOCKS_PER_SEC 1000
//#endif

namespace dart {
namespace common {

double subtractTimes( double endTime, double startTime) {
    return (endTime - startTime) / CLOCKS_PER_SEC;
}

Timer::Timer(const char* name) {
    mCount = 0;
    mTotal = 0;
    mName = new char[64];
    strcpy(mName, name);
    mIsRunning = false;
#if WIN32
    mTimer.start.QuadPart=0;
    mTimer.stop.QuadPart=0;
    QueryPerformanceFrequency( &mFrequency ) ;
#endif
}

Timer::~Timer() {
    print();
    delete[] mName;
}

#if WIN32
double Timer::LIToSecs( LARGE_INTEGER & L) {
    return ((double)L.QuadPart /(double)mFrequency.QuadPart) ;
}
#endif

void Timer::startTimer() {
    mIsRunning = true;
    mCount++;
#if WIN32
    QueryPerformanceCounter(&mTimer.start) ;
#else
    mStart = clock();
#endif
}

double Timer::elapsed() {
#if WIN32
    LARGE_INTEGER timenow;
    QueryPerformanceCounter(&timenow) ;
    LARGE_INTEGER time;
    time.QuadPart = timenow.QuadPart - mTimer.start.QuadPart;
    mLastElapsed = LIToSecs( time) ;
#else
    double now = clock();
    mLastElapsed = subtractTimes(now, mStart);
#endif
    return mLastElapsed;
}

double Timer::lastElapsed() const {
    return mLastElapsed;
}

double Timer::total() const {
    return mTotal;
}

void Timer::stopTimer() {
    mIsRunning = false;
#if WIN32
    QueryPerformanceCounter(&mTimer.stop) ;
    LARGE_INTEGER time;
    time.QuadPart = mTimer.stop.QuadPart - mTimer.start.QuadPart;
    mLastElapsed = LIToSecs( time) ;
#else 
    mStop = clock();
    mLastElapsed = subtractTimes(mStop, mStart);
#endif
    mTotal += mLastElapsed;
}

bool Timer::isRunning() const {
    return mIsRunning;
}

void Timer::printLog() {
    if(mCount>0) {
        std::cout << "[VLOG(1)]" << "Timer [" << mName << "] : "
             << "Last = " << mLastElapsed << " "
             << "Total " << " "
             << mTotal << " " << mCount << "; "
             << "Average: " << mTotal / mCount << " "
             << "FPS : " << mCount / mTotal << "hz ";

    } else {
        std::cout << "[VLOG(1)]" << "Timer " << mName << " doesn't have any record." << std::endl;
    }
}

void Timer::printScreen() {
    if(mCount>0) {
        std::cout << "Timer [" << mName << "] : "<<std::endl
             << "Last elapsed : " << mLastElapsed << "; "
             << "Total time : " << " "
             << mTotal << "; "
             << "Total count : " << mCount << "; "
             << "Average time : " << mTotal / mCount << " "
             << "FPS : " << mCount / mTotal << "hz "<<std::endl;

    } else {
        std::cout << "Timer " << mName << " doesn't have any record." << std::endl;
    }
}

void Timer::print( bool _toScreen ){
    if( _toScreen ) printScreen();
    else printLog();
}

} // namespace common
} // namespace dart
