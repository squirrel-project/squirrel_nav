/*
 * 6D localization for humanoid robots
 *
 * Copyright 2009-2012 Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/humonoid_localization
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <squirrel_3d_localizer/SquirrelLocalizer.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "squirrel_3d_localizer");

  ros::NodeHandle private_nh("~");
  unsigned seed;
  int iseed;
  private_nh.param("seed", iseed, -1);
  if (iseed == -1)
    seed = static_cast<unsigned int>(std::time(0));
  else
    seed = static_cast<unsigned int>(iseed);

  squirrel_3d_localizer::SquirrelLocalizer localization(seed);

  ros::spin();

  return EXIT_SUCCESS;
}
