Data we care about in the lidar bag file:

This message includes the point cloud data
- Type: sensor_msgs/Image
/device_0/sensor_0/Depth_0/image/data

This message includes the color image
- Type: sensor_msgs/Image
/device_0/sensor_1/Color_0/image/data

This message includes the IR image data
- Type: sensor_msgs/Image
/device_0/sensor_0/Infrared_0/image/data


Dependencies to build catkin on a linux computer:
librealsense
ros (I am running with melodic)
cmake

To build:
cd to MyCatkin/App
run 'catkin_make' 
If it shows getting to 100% you are golden

To run post processing steps to get desire number of points 
out of a recorded bag file, ensure you have updated the following
in the pointcloud.cpp file in main:
- depthPoints.upperLeft pixels
- depthPoints.upperRight pixels
- file path



