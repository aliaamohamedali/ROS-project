#!/bin/bash
echo "Starting Benzo"
## roscore &
roslaunch turtlebot_bringup minimal.launch &
roslaunch rtabmap_ros demo_turtlebot_mapping.launch args:="--delete_db_on_start" rgbd_odometry:=true
## roslaunch rtabmap_ros demo_turtlebot_rviz.launch
## rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600 &
python ~/workspaces/grad_project_catkin_workspace/src/App-Server/Server.py &
python ~/workspaces/grad_project_catkin_workspace/src/yolo_classify/src/yolo_classify.py
