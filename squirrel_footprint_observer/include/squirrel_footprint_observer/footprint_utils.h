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

#ifndef SQUIRREL_FOOTPRINT_OBSERVER_FOOTPRINT_UTILS_H_
#define SQUIRREL_FOOTPRINT_OBSERVER_FOOTPRINT_UTILS_H_

#include "squirrel_footprint_observer/geometry_types.h"

#include <cmath>
#include <vector>

namespace squirrel_footprint_observer {

template <int NumPoints>
void approximatedCircle(double radius, std::vector<Point2D>* polygon) {
  static_assert(NumPoints > 0, "Number of points must be positive");
  // Create the approximated circle.
  polygon->clear();
  polygon->reserve(NumPoints);
  const double da = 2 * M_PI / NumPoints;
  for (int i = 0; i < NumPoints; ++i) {
    const double a = i * da;
    polygon->emplace_back(radius * std::cos(a), radius * std::sin(a));
  }
}

}  // namespace squirrel_footprint_observer

#endif /* SQUIRREL_FOOTPRINT_OBSERVER_FOOTPRINT_UTILS_H_ */
