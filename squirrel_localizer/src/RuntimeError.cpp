// RuntimeError.cpp --- 
// 
// Filename: RuntimeError.cpp
// Description:  
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 14:04:31 2015 (+0100)
// Version: 
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
//   ROS Hydro
//   ROS Indigo
// 

// Code:


#include "squirrel_localizer/RuntimeError.h"
#include "squirrel_localizer/OSSpecific.h"

#include <cstdarg>
#include <cstdlib>
#include <cstdio>

using namespace std;

RuntimeError::RuntimeError(const char* fmt, ...) :
  std::exception()
{
  char* auxPtr = NULL;
  va_list arg_list;
  va_start(arg_list, fmt);
  int b = vasprintf(&auxPtr, fmt, arg_list);
  va_end(arg_list);
  if (b > 0)
    _errorMsg = auxPtr;
  else
    _errorMsg = "";
  free(auxPtr);
}

RuntimeError::~RuntimeError() throw()
{
}

// 
// RuntimeError.cpp ends here
