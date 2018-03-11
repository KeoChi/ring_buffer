# ring_buffer

Currently this project contains two ROS package, cg_mapping and cg_ring_buffer

## cg_mapping
This package performs simple mapping, with synchronized pointcloud and camera pose as **input** and transformed pointcloud as **output**. It publish pointcloud every 0.5 second.

Usage:

Firstly run:
> rosrun cg_mapping simple_mapping

Then open rviz with the **cloud.rviz** file. If everything runs correctly, you should see pointcloud in rviz. Please make sure that topics of *PointCloud* and *Pose* are correctly set in *simple_mapping.cpp*

## cg_ring_buffer

This package is a implement of RingBufferMap, which is always centralized around the camera(robot), and contains occupancy, signed distance and pointcloud normal information. It take pointcloud as input and update the RingBufferMap. 
 