squirrel_footprint_observer
===========================

Update the footprint of the robot according to arm configuration.

![snapshot](https://github.com/federico-b/squirrel_nav/blob/indigo_dev/squirrel_footprint_observer/doc/snapshot.png)

### Nodes

The package provide a single node

- `squirrel_footprint_observer_node`: Run the footprint observer.

### Parameters
- `~/base_frame_id`: The frame of the robot base.
- `~/base_radius`: The radius of the robot base (if circular).
- `~/joint_chain`: The chain of frames realated to the arm.
- `~/joint_radius`: The radius of a sphere circumscribing the arm joint.

See `robotino_footprint.yaml` as example.

### Advertised Services
Uses messages provieded by [squirrel_footprint_observer_msgs](https://github.com/squirrel-project/squirrel_common/tree/indigo_dev/squirrel_footprint_observer_msgs)
- `~/getFootprint`: Retrieve the current footprint of the robot.
- `~/dumpFootprint`: Print footprint on file or stdout/stderr as well
  as on a file.

### Advertised Topics
- `~/footprint`(*geometry_msgs/PolygonStamped*): The current footprint.

### Subscriptions
- `/tf`: the transformations from `base_frame_id` to the frames
  specified in `joint_chain`.
- `~/enable` (*std_msgs/Bool*): Toggle the constant update of the
  footprint observer. Disable `~/footprint` to be published while the
  service `~/getFootprint` remains active.
