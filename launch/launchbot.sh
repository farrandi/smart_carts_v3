#!/bin/bash

echo "Hello, world!"
echo "Launching new terminals"

# Ways to open up a terminal and have it run a command
# gnome-terminal -- command
# xterm -e -hold command
# konsole -e --noclose command

# So assuming we want to use xterm

# These can be run from anywhere
xterm -e roscore &
xterm -e rosrun rosserial_python serial_node.py /dev/ttyACM0 &
roslaunch realsense2_camera rs_camera.launch align_depth:=true --screen &

# These need to be run in the correct directory, catkin_ws/src/SmartCartsV2/launch

roslaunch target_pose_tracker.launch --screen &
roslaunch target_follow.launch --screen

# Note that you can have a launch file launch multiple files, so this should really be a launch file instead of a base script