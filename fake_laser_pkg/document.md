# Guide 
The node launches the camera driver "Freenect" automatically
Use Roboviz to view the camera and make sure that the following is selected to view the sensor_msgs/LaserScan:

![Image from Mastering ROS for Robotics Programming]()


* Set Fixed Frame as camera_depth_frame 
* Add the LaserScan in topic /scan.




Note: If the Camera Node gets suspended or stopped, use this in the terminal before using roslaunch
$ sudo sh -c "echo -1 > /sys/module/usbcore/parameters/autosuspend"
