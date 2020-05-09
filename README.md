# ROS-project
ROS nodes and packages for building an autonomous driving robot.

## Folders

### kinect
This folder contains the packages for drivers and/or nodes responsible for handling Kinect images (mainly the kinect driver)

### Launch
This folder contains launch files for running different project components (mapping, localization and navigation etc..)

### maps
This folder contains any generated maps we need.

### odom
This folder contains packages for drivers and/or nodes responsible for handling odometry data (imu & encoder readings)

### sensors
This folder contains packages for drivers and/or nodes responsible for handling laser sensor data (LIDAR or any proxy pkg eg: kinect based)
For now we try to sustitute Laser_scan data based on laser sensors with a kinect RGB image based estimator

### transforms
This folder contains any tf::Broadcasters for updating links between components and translating readings between frames
Primarily this should have a:
-> odom->base_footer broadcaster
-> base_link->kinect broadcaster and translater
-> base_link->arm broadcaster and translater

### urdf
This folder contains description files for different components and static tf trees (Robot and wheels/sensors, kinect etc..)