#!/bin/bash

echo "Killing all ROS nodes..."
killall -INT roslaunch
killall -INT roscore
killall -INT rosnode
echo "All ROS nodes killed."