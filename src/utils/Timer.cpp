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
    using namespace boost::posix_time;
  
    double subtractTimes( ptime endTime, ptime startTime) {
        static const double micro_in_one_sec = static_cast<double>(
            time_duration(seconds(1)).total_microseconds());
        double elapsed = static_cast<double>(
            (endTime - startTime).total_microseconds()
            / micro_in_one_sec);
        return elapsed;
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
        mStart = microsec_clock::local_time();
    }

    double Timer::elapsed() {
        ptime now = microsec_clock::local_time();
        mLastElapsed = subtractTimes(now, mStart);
        return mLastElapsed;
    }

    double Timer::lastElapsed() const {
        return mLastElapsed;
    }

    void Timer::stopTimer() {
        mIsRunning = false;
        mStop = microsec_clock::local_time();
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

    scoped_timer::scoped_timer(Timer* _timer)
        : timer_(_timer) {
        timer()->startTimer();
    }

    scoped_timer::~scoped_timer() {
        timer()->stopTimer();
    }

    Timer* scoped_timer::timer() const {
        return timer_;
    }

} // namespace utils
