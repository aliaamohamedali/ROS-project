# Odom Package

* This package is for forming Odometry data in the absence of wheel encoders from hardware.
* It utilizes imu sensor data to construct a nav_msgs::Odometry message

## Work Flow
* The package consists of one executable. Which listens to incoming sensor data from an imu (mpu6050) sensor and converts them to odometry data
* The package uses Newton's laws and kinematics to convert linear acceleration & angular velocity data from sensor to post and twist information
* To reduce the accumulated errors whih result from only relying on the imu sensor data. we add more parameters to improve approximate computing velocity from acceleration
* For now it is only jerk (rate of change of acceleration)

## Odom-Publisher

### Arguments
* None

### Publishers
**<nav_msgs/Odometry> publisher**
* This publisher publishes to a message topic called "odom"

### Subscribers
**<sensor_msgs/Imu> subscriber**
* This subscriber listens to messages on a topic called "imu"


## How to use
Clone this folder in the /src directory of a catkin workspace.\
Run "catkin_make"\
Source the setup.bash file in the /devel directory of the workspace

Run rosrun transforms <file_name> and pass in the required arguments

(You can find the name ro run an executable in the package.xml file)