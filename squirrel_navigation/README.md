squirrel_navigation
===================

Autonomous navigation for SQUIRREL

## Libraries, nodes and plugins

- `squirrel_navigation_utils`: Utility library used in the package.
- `squirrel_navigation_safety`: Safety tools for navigation.
- `squirrel_navigation_planners`: Local and global planners.
- `squirrel_navigation_costmap_layer`: Costmap layer used for
  navigation.

## SQUIRREL Planners

Contains `squirrel_navigation::GlobalPlanner`,
`squirrel_navigation::LocalPlanner` used for 2D navigation. 

## SQUIRREL Costmap layers

Contains `squirrel_navigation::NavigationLayer` which merges obstacles
detected with depth camera and the safety laser rangefinder.
