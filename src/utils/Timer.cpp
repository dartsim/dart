/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include <iostream>
using namespace std;
// Google Libraries
#include <glog/logging.h>
using namespace google;
#include "Timer.h"  // for WIN32 placing this before glog/logging.h causes compilation errors

namespace utils {
  
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

    void Timer::print() {
        if(mCount>0) {
            VLOG(1) << "Timer [" << mName << "] : "
                    << "Last = " << mLastElapsed << " "
                    << "Total " << " "
                    << mTotal << " " << mCount << "; "
                    << "Average: " << mTotal / mCount << " "
                    << "FPS : " << mCount / mTotal << "hz ";
       
        } else {
            VLOG(1) << "Timer " << mName << " doesn't have any record." << endl;
        }
    }

} // namespace utils
