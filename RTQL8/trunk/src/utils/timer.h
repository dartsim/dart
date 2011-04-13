#ifndef SRC_UTILS_TIMER_H
#define SRC_UTILS_TIMER_H

#include <boost/date_time/posix_time/posix_time.hpp>
namespace bp = boost::posix_time;

namespace utils {
  class Timer;
  class scoped_profile;


  /**
     @brief The implementation of Timer class

     This is a definition of timer class.
     For measure the time, boost date_time is used.
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
    bp::ptime mStart;
    bp::ptime mStop;
    double mLastElapsed;
    double mTotal;
    char* mName;
    bool bRunning;
  };

  /**
     @brief A helper class for timer class
     @see Timer

  */
  class scoped_timer {
  public:
    scoped_timer(Timer* timer);
    ~scoped_timer();
    Timer* timer() const;
  private:
    Timer* timer_;
  };
} // namespace utils

#endif // #ifndef SRC_UTILS_TIMER_H
