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

#ifndef SQUIRREL_FOOTPRINT_OBSERVER_GEOMETRY_TYPES_H_
#define SQUIRREL_FOOTPRINT_OBSERVER_GEOMETRY_TYPES_H_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>

namespace squirrel_footprint_observer {

typedef CGAL::Exact_predicates_inexact_constructions_kernel::Point_2 Point2D;
typedef CGAL::Exact_predicates_inexact_constructions_kernel::Vector_2 Vector2D;

}  // namespace squirrel_footprint_observer

#endif /* SQUIRREL_FOOTPRINT_OBSERVER_GEOMETRY_TYPES_H_ */
