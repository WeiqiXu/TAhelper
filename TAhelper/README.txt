
/***********************************************/
Package name: TAhelper
Authors: Weiqi XU, wex064@eng.ucsd.edu
	 Yiding Qiu, yiqiu@eng.ucsd.edu
Date: 03/17/2019
Description: This is a ROS package for turtlebot that checks students attendance, and 		     deliver the homework to TA. CMvision, face_recognition, turtlebot_navigation 	     and sound_play package are used to complete this task.
/***********************************************/

The executable file is TAhelper.cpp under src folder.
To get the program running, simply do:

>roscore
>roslaunch turtlebot_bringup minimal.launch

>rosrun sound_play soundplay_node.py

>roscd usb_cam
>rosparam set usb_cam/pixel_format yuyv 
>rosrun usb_cam usb_cam_node 
>rosrun image_view image_view image:=/usb_cam/image_raw
>roslaunch cmvision colorgui image:=/camera/rgb/image_raw
<this is for color calibration, close when done>
>roslaunch cmvision cmvision.launch image:=/usb_cam/image_raw
<cnrl-c to kill process>
>rosparam set /cmvision/color_file ~/turtlebot_ws/src/cmvision/colors.txt
>rosrun cmvision cmvision image:=/usb_cam/image_raw

>roscd face_recognition
>rosrun face_recognition Fserver

>export TURTLEBOT_MAP_FILE=/tmp/my_map.yaml
>roslaunch turtlebot_navigation amcl_demo.launch

>rosrun TAhelper TAhelper


Note that:
1. To detect the people you want, you need to build and train your own face_recognition dataset. For more details, please refer to: http://wiki.ros.org/face_recognition 
1. The source code is written up to only take bright yellow color patch as the homework marker. To change the color or environment, re-calibration of colors is needed. The RGB values and the corresponding YUV values need to be stored in colors.txt file.
2. The map for SLAM need to be saved in my_map.png For more details, please refer to http://wiki.ros.org/turtlebot_navigation/Tutorials
