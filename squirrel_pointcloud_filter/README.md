squirrel_pointcloud_filter
==========================

Simple utility to downsample pointclouds.

### Nodes

The package provides a single node
- `squirrel_pointcloud_filter_node`


### Parameters
- `~/global_frame_id` The world reference frame.
- `~/update_rate_hz` The spinning frequency of the node.
- `~/nanfree` Whether the input pointcloud is NaN free.
- `~/do_voxel_filter` Whether to apply or not voxel filtering 
- `~/do_ground_segmentation` Segment ground/nonground points.
- `~/ground_pcls_voxelized` Whether to use the voxelized pointcloud for
  the ground segmentation.
- `~/ground_threshold` Tolerance to accept points in the ground.
- `~/resolution_{x, y, z}` The resolution of the voxel filter.

### Advertised Topics

- `~/{voxelized, ground, nonground}_cloud_out` The resulting pointclouds.
