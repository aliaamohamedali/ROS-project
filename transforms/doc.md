# Transforms Package

* This package is for broadcasting static and dynamic transforms between frames.
* For static transforms it mainly takes them as input
* For dynamic transforms it subscribes to necessary topics, makes some middle computations then broadcasts. May also make use of passed in input

## Work Flow

* Currently the package has two executables. One for publishing camera and body static transforms and another for publishing odometry transform (between odom and body)
* The camera-body transform takes 3 arguments, in order the transform in x-axis, y-axis (linear) and its rotation with respect to each other in z-axis.

## Camera-Body Broadcaster

### Arguments
* Three Arguments
* the first is for translation in x-axis
* the 2nd   is for translation in y-axis
* the last  is for (rotational) translation in z-axis  

### Broadcasters
* One broadcaster that publishes transform between "camera" and "base_link" frame links
* "base_link" is the parent frame
* "camera" is the child frame

## Body-Odom Broadcaster

### Arguments
* None

### Subscribers
**<nav_msgs/Odometry> subscriber**
* Subscribes to the message on a topic called "odom"

### Broadcasters
* One broadcaster that publishes transform between "base_link" and "odom" frame links
* "odom" is the parent frame
* "base_link" is the child frame

The broadcaster uses the odometry data it recieves on the odom topic to continuously adjust the transform

## How to use
Clone this folder in the /src directory of a catkin workspace.\
Run "catkin_make"\
Source the setup.bash file in the /devel directory of the workspace

Run rosrun transforms <file_name> and pass in the required arguments

(You can find the name ro run an executable in the package.xml file)

