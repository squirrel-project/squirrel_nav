// OSSpecific.h --- 
// 
// Filename: OSSpecific.h
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

#ifndef SQUIRREL_LOCALIZER_OSSPECIFIC_H_
#define SQUIRREL_LOCALIZER_OSSPECIFIC_H_

#ifdef WINDOWS

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/time.h>

typedef unsigned int uint;

#define drand48() ((double) rand()/(double)RAND_MAX)

#ifdef __cplusplus
extern "C" {
#endif

int vasprintf(char** strp, const char* fmt, va_list ap);

#ifdef __cplusplus
}
#endif

#endif

#ifdef LINUX

#include <sys/time.h>

#endif

#endif /* SQUIRREL_LOCALIZER_OSSPECIFIC_H_ */

//
// OSSpecific.h ends here
