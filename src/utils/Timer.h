/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef UTILS_TIMER_H
#define UTILS_TIMER_H

#if WIN32
#include <windows.h>
typedef struct {
    LARGE_INTEGER  start;
    LARGE_INTEGER  stop;
} stopWatch;
#endif

namespace utils {
    class Timer;

    /**
       @brief The implementation of Timer class

       This is a definition of mTimer class.
       For measure the time, clock() api is used
    */
    class Timer {
    public:
        Timer(const char* name); ///< Default constructor. The name can be up to 64
        ~Timer(); ///< Default destructor
	
        void startTimer(); 
        double elapsed(); ///< return elapsed time in seconds since startTimer().
        ///< @see startTimer()
        double lastElapsed() const;
        void stopTimer();
        bool isRunning() const;
        void print();

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
} // namespace utils

#endif // #ifndef UTILS_TIMER_H
