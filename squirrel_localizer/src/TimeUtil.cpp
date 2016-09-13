// TimeUtil.cpp --- 
// 
// Filename: TimeUtil.cpp
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 14:06:08 2015 (+0100)
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

#include "squirrel_localizer/TimeUtil.h"

#include <iostream>

std::string getTimeAsString(time_t time)
{
  std::string dateStr = ctime(&time);
  if (dateStr.size() == 0)
    return "";
  // remove trailing newline
  dateStr.erase(dateStr.size()-1, 1);
  return dateStr;
}

std::string getCurrentTimeAsString()
{
  return getTimeAsString(time(NULL));
} 


ScopeTime::ScopeTime(const char* title) : _title(title), _startTime(get_time()) {}

ScopeTime::~ScopeTime() {
  std::cerr << _title<<" took "<<1000*(get_time()-_startTime)<<"ms.\n";
}

// TimeUtil.cpp ends here
