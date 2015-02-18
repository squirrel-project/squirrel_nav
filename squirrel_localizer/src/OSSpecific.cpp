// OSSpecific.cpp --- 
// 
// Filename: OSSpecific.c
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 14:02:03 2015 (+0100)
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


#include "squirrel_localizer/OSSpecific.h"

#ifdef WINDOWS

int vasprintf(char** strp, const char* fmt, va_list ap){
  int n;
  int size = 100;
  char* p;
  char* np;

  if ((p = (char*)malloc(size * sizeof(char))) == NULL)
    return -1;

  while (1) {
    n = vsnprintf(p, size, fmt, ap);
    if (n > -1 && n < size) {
      *strp = p;
      return n;
    }
    if (n > -1)
      size = n+1;
    else
      size *= 2;
    if ((np = (char*)realloc (p, size * sizeof(char))) == NULL) {
      free(p);
      return -1;
    } else
      p = np;
  }
}

#endif

// 
// OSSpecific.c ends here
