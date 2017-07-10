// The MIT License (MIT)
//
// Copyright (c) 2017 Federico Boniardi and Wolfram Burgard
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef SQUIRREL_FOOTPRINT_OBSERVER_FOOTPRINT_IO_H_
#define SQUIRREL_FOOTPRINT_OBSERVER_FOOTPRINT_IO_H_

#include <geometry_msgs/Point32.h>

#include <iostream>
#include <vector>

namespace squirrel_footprint_observer {

inline std::ostream& operator<<(
    std::ostream& os, geometry_msgs::Point32& point) {
  os << "[" << point.x << ", " << point.y << "]";
  return os;
}

inline std::ostream& operator<<(
    std::ostream& os, const std::vector<geometry_msgs::Point32>& points) {
  size_t i = 1;
  os << "[";
  for (const auto& point : points) {
    os << point;
    if (i < points.size())
      os << ", ";
  }
  os << "]";
  return os;
}

}  // namespace squirrel_footprint_observer

#endif /* SQUIRREL_FOOTPRINT_OBSERVER_FOOTPRINT_IO_H_ */
