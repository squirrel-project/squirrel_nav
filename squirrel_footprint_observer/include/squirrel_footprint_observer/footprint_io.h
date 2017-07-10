// Copyright (C) 2017  Federico Boniardi and Wolfram Burgard
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
