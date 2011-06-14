/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "Timer.h"
#include <iostream>
using namespace std;
// Google Libraries
#include <glog/logging.h>
using namespace google;

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
    }

    Timer::~Timer() {
        print();
        delete[] mName;
    }

    void Timer::startTimer() {
        mIsRunning = true;
        mCount++;
        mStart = clock();
    }

    double Timer::elapsed() {
        double now = clock();
        mLastElapsed = subtractTimes(now, mStart);
        return mLastElapsed;
    }

    double Timer::lastElapsed() const {
        return mLastElapsed;
    }

    void Timer::stopTimer() {
        mIsRunning = false;
        mStop = clock();
        mLastElapsed = subtractTimes(mStop, mStart);
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
