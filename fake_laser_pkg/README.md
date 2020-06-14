# Guide 
This node package launches the camera driver "Freenect" automatically
Use Roboviz to view the camera and make sure that the following is selected to view the sensor_msgs/LaserScan:

* $ roslaunch fake_laser_pkg laser_start.launch
* $ rosrun rviz rviz

![Image from Mastering ROS for Robotics Programming](https://github.com/aliaamohamedali/ROS-project/blob/master/photos/roslaser.png)

Refrence: Mastering ROS for Robotics Programming


* Set Fixed Frame as camera_depth_frame 
* Add the LaserScan in topic /scan.




Note: If the Camera Node gets suspended or stopped, use this in the terminal before using roslaunch
$ sudo sh -c "echo -1 > /sys/module/usbcore/parameters/autosuspend"
