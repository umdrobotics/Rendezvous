#!/bin/bash


#echo("Run this AFTER \"sudo su\" . Start roscore before running this script, and start the console applications after")


ROS_NAMESPACE=dji_sdk rosrun image_proc image_proc &
roslaunch apriltags_ros apriltags_ros-test.launch &
roslaunch dji_sdk_read_cam manifold_cam.launch &

sleep 5; # small delay in seconds to let everything stabilize

#now initialize communication with the quadcopter
roslaunch dji_sdk sdk_manifold.launch; 

