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

#ifndef DART_COMMON_TIMER_H
#define DART_COMMON_TIMER_H

#if WIN32
#define NOMINMAX
#include <windows.h>
typedef struct {
    LARGE_INTEGER  start;
    LARGE_INTEGER  stop;
} stopWatch;
#endif

namespace dart {
namespace common {

/// @brief The implementation of Timer class
///
/// This is a definition of mTimer class.
/// For measure the time, clock() api is used
class Timer {
public:
    /// @brief Default constructor. The name can be up to 64.
    Timer(const char* name = "noname");

    /// @brief Default destructor.
    ~Timer();

    /// @brief
    void startTimer();

    /// @brief Return elapsed time in seconds since startTimer().
    /// @see startTimer()
    double elapsed();

    /// @brief
    double lastElapsed() const;

    /// @brief
    double total() const;

    /// @brief
    void stopTimer();

    /// @brief
    bool isRunning() const;

    /// @brief
    void printLog();

    /// @brief
    void printScreen();

    /// @brief
    void print(bool _toScreen = true);

private:
    int mCount;

#if WIN32
    stopWatch mTimer;
#else
    double mStart;
    double mStop;
#endif
    double mLastElapsed;
    double mTotal;
    char *mName;
    bool mIsRunning;

#if WIN32
    LARGE_INTEGER  mFrequency;
    double LIToSecs( LARGE_INTEGER & L) ;
#endif

};

} // namespace common
} // namespace dart

#endif // #ifndef DART_COMMON_TIMER_H
