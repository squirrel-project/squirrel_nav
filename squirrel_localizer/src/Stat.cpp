// Stat.cpp --- 
// 
// Filename: Stat.cpp
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 13:37:17 2015 (+0100)
// Version: 0.1.0
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


#include "squirrel_localizer/Stat.h"

#include <cmath>
#include <cstdlib>
#include <assert.h>

#include "squirrel_localizer/OSSpecific.h"

namespace AISNavigation{

  double triangularSample(double width, double mean){
    double u=drand48();
    if (u<=0.5)
      u=-0.5+sqrt(u*0.5);
    else
      u=0.5-sqrt((1-u)*0.5);
    return u*2*width+mean;
  };


}; // namespace AISNavigation

// 
// Stat.cpp ends here
