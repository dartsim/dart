/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef UTILS_TIMER_H
#define UTILS_TIMER_H

namespace utils {
    class Timer;

    /**
       @brief The implementation of Timer class

       This is a definition of timer class.
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
        double mStart;
        double mStop;
        double mLastElapsed;
        double mTotal;
        char *mName;
        bool mIsRunning;
    };
} // namespace utils

#endif // #ifndef UTILS_TIMER_H
