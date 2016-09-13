// FloatMap.h --- 
// 
// Filename: FloatMap.h
// Description: 
// Author: Joerg Roewenkaemper
// Maintainer: boniardi@cs.uni-freiburg.de
// Created: Tue Feb 17 14:19:42 2015 (+0100)
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

#ifndef SQUIRREL_LOCALIZER_FLOATMAP_HH_
#define SQUIRREL_LOCALIZER_FLOATMAP_HH_

#include "squirrel_localizer/GridMap.h"

#include <iostream>
#include <vector>

#include <stdio.h>
#include <stdint.h>

namespace AISNavigation {

struct FloatMap: public _GridMap<float> {
  friend struct FrequencyMap;
  FloatMap():_GridMap<float>(){}
  FloatMap(const Vector2i& size, double resolution, const Vector2& offset, float unknown);
  void computeDistanceMap(FloatMap& dmap, double maxDistance, double minOccupancy, bool markUnknown) const;
  void saveAsPPM(std::ostream& os, bool equalize=false) const;
  bool loadFromPPM(std::istream& is, double res=0.05);
  bool fromInt8Vector(const uint32_t& size_x_,const uint32_t& size_y_,
                      const double& offset_x_, const double&  offset_y_,
                      const float& resolution_, const std::vector<signed char>& data );
  /*bool fromInt8Vector(const uint32_t size_x_,const uint32_t size_y_,
    const uint32_t offset_x_, const uint32_t  offset_y_,
    const float resolution_, std::vector<int> data );*/
  int toChar4Array(unsigned char* buffer, bool equalize=false) const; //for visualization
};

}

#endif  /* SQUIRREL_LOCALIZER_FLOATMAP_H_ */

//
// FloatMap.h ends here
