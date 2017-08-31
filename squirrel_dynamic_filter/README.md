squirrel_dynamic_filter
=======================
This package is used for removing the dynamic points from the scan and for classification of the scene into movable and non-movable objects

The package provides different nodes:

    dynamic_filter_node: for estimating point wise motion

    StaticClassifyOctomap: for classifying the non-movable parts of the scene

    preprocessing: it is the main node, which removes the ground, call the

    StaticClassifyOctomap and dynamic_filter_node

    TemporalInference: for classifying points as movable or dynamic It requires the kinect point cloud and the octomap of the environment

###Parameters

    Parameters can be accessed at params/parameters.yaml file

###Dependices

    g2o: installed in the external folder
    mlpack: installed in the external folder
    libarmadillo: sudo apt-get install libarmadillo-dev -y

###Usage

    roslaunch squirrel_dynamic_filter dynamic_filter.launch
    roslaunch squirrel_dynamic_filter dynamic_filter_node.launch

###Output Topics

    /kinect/depth/static - pointcloud after filtering dynamic objects and the new static objects
    /octomap_full_color - colorized octomap. Green color corresponds to non-movable objects and blue color corresponds to movable objects



