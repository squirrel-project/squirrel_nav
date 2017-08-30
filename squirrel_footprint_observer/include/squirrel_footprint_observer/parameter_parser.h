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

#ifndef SQUIRREL_FLOORPLAN_LOCALIZER_PARAMETER_PARSER_H_
#define SQUIRREL_FLOORPLAN_LOCALIZER_PARAMETER_PARSER_H_

#include "squirrel_footprint_observer/geometry_types.h"

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace squirrel_footprint_observer {
namespace parameter_parser {

inline bool parseFootprintParameter(
    const std::string& footprint_str, std::vector<Point2D>* footprint) {
  const YAML::Node& footprint_node = YAML::Load(footprint_str);
  if (footprint_node.size() <= 0.)
    return false;
  footprint->clear();
  footprint->reserve(footprint_node.size());
  for (size_t i = 0; i < footprint_node.size(); ++i) {
    const YAML::Node& point_node = footprint_node[i];
    if (point_node.size() <= 0.)
      return false;
    footprint->emplace_back(
        point_node[0].as<float>(), point_node[1].as<float>());
  }
  return true;
}

}  // namespace parameter_observer
}  // namespace squirrel_footprint_parser

#endif /* SQUIRREL_FLOORPLAN_LOCALIZER_PARAMETER_PARSER_H_ */
