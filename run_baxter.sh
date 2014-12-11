#!/usr/bin/python

echo "Closing all cameras...."
rosrun baxter_tools camera_control.py -c left_hand_camera
rosrun baxter_tools camera_control.py -c right_hand_camera
rosrun baxter_tools camera_control.py -c head_camera

echo "Turning on right hand camera..."
rosrun baxter_tools camera_control.py -o right_hand_camera
rosrun baxter_tools camera_control.py -o left_hand_camera

echo "Wake Up Eyes"
rosrun baxter_examples xdisplay_image.py --file=/home/lin_tang_wai/ros_workspace/eyes1.jpg 

echo "Enable Robot"
rosrun baxter_tools enable_robot.py -e

echo "Running Giant Launch File"
roslaunch ik run_baxter.launch

rosrun ik MAIN.py
