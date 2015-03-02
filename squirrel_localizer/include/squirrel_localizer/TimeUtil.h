// TimeUtil.h --- 
// 
// Filename: TimeUtil.h
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 14:11:01 2015 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
//    ROS Hydro
//    ROS Indigo
// 

// Code:

#ifndef SQUIRREL_LOCALIZER_TIMEUTIL_H_
#define SQUIRREL_LOCALIZER_TIMEUTIL_H_

#include <sys/time.h>

#include <string>

/** @addtogroup utils **/
// @{

/** \file timeutil.h
 * \brief utility functions for handling time related stuff
 */

/// Executes code, only if secs are gone since last exec.
/// extended version, in which the current time is given, e.g., timestamp of IPC message
#ifndef DO_EVERY_TS
#define DO_EVERY_TS(secs, currentTime, code)    \
  if (1) {                                      \
    static double s_lastDone_ = (currentTime);  \
    double s_now_ = (currentTime);              \
    if (s_lastDone_ > s_now_)                   \
      s_lastDone_ = s_now_;                     \
    if (s_now_ - s_lastDone_ > (secs)) {        \
      code;                                     \
      s_lastDone_ = s_now_;                     \
    }                                           \
  } else                                        \
    (void)0
#endif

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY
#define DO_EVERY(secs, code) DO_EVERY_TS(secs, get_time(), code)
#endif

#ifndef MEASURE_TIME
#define MEASURE_TIME(text, code)                                        \
  if(1) {                                                               \
    double _start_time_ = get_time();                                   \
    code;                                                               \
    fprintf(stderr, "%s took %f sec\n", text, get_time() - _start_time_); \
  } else                                                                \
    (void) 0
#endif

/**
 * return the current time in seconds since 1. Jan 1970
 */
inline double get_time() {
  struct timeval ts; 
  gettimeofday(&ts,0);
  
  return ts.tv_sec + ts.tv_usec*1e-6;
}

/**
 * return the string of a given time
 */
std::string getTimeAsString(time_t time);

/**
 * return the default string of the current time
 */
std::string getCurrentTimeAsString();

/**
 * \brief Class to measure the time spent in a scope
 *
 * To use this class, e.g. to measure the time spent in a function,
 * just create and instance at the beginning of the function.
 */
class ScopeTime {
 public: 
  ScopeTime(const char* title);
  ~ScopeTime();
 private:
  std::string _title;
  double _startTime;
};

#define MEASURE_FUNCTION_TIME                   \
  ScopeTime scopeTime(__PRETTY_FUNCTION__)


// @}
#endif /* SQUIRREL_LOCALIZER_TIMEUTIL_H_ */

//
// TimeUtil.h ends here
