# ring_buffer

This project contains several ROS packages, simple_mapping and ewok_\*, and keyboard control.
- _simple\_mapping_ is used to perform simple point cloud stitching. 
- _ewok_ is a modified version of the _ewok_ project, which contains ring buffer map, spline trajectory representation and fast replanning
- _keyboard\_control_ read keyboard value and publish control message.

 You can clone the whole project to your *catkin_ws/src* and then *catkin_make* it.

```
cd <SOME_PATH>/catkin_ws/src

git clone https://github.com/ZbyLGsc/ring_buffer.git

cd ..

catkin_make
```

This will build both packages altogether.


## simple_mapping
This package performs simple mapping, with synchronized pointcloud and camera pose as **input** and transformed pointcloud as **output**. It publish pointcloud every 0.5 second.

Usage:

Firstly run:
> rosrun simple_mapping simple_mapping

Then open rviz with the **cloud.rviz** file. If everything runs correctly, you should see pointcloud in rviz. Please make sure that topics of *PointCloud* and *Pose* are correctly set in *simple_mapping.cpp*

## ewok_ring_buffer

This package is a implement of RingBufferMap, which is always centralized around the camera(robot), and contains occupancy, signed distance and pointcloud normal information. 

Usage:

```
roslaunch ewok_ring_buffer ring_buffer.launch

rviz -d ring_buffer.rviz

```
Run *ring_buffer.launch*, open *ring_buffer.rviz* and feed synchronized pointcloud and odometry as input. Please make sure that topics of *PointCloud* and *Odometry* are correctly set. 

The ring_buffer map is published as *visualization\_msgs::Marker*, as well as *PointCloud2*. The center of ring\_buffer map is published as *geometry\_msgs::PoseStamped*. They are synchronized using *header.stamp*, so you can use
*message\_filters* to subscribe them.

Note: this readme is __not__ up-to-date. More information will be added.

 