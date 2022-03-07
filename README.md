# tello_visual_odometry
Navigation code for tello edu drone using visual odometry and the AR Toolkit

## Ubuntu Setup (Using Anaconda):
1. conda create -n tello 
2. conda activate tello
3. pip install -r requirements.txt
4. sudo apt install libswscale-dev libavcodec-dev libavutil-dev
5. cd h264decoder
6. pip install . 

## Catkin Workspace setup
1. source /opt/ros/noetic/setup.bash
2. mkdir -p ~/catkin_ws/src
3. cd ~/catkin_ws/
4. catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
5. cd src
6. catkin_create_pkg tello_visual_odom std_msgs rospy roscpp


## Camera Calibration:
1.  rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/camera/image_raw

## Using Calibration:
1. ROS_NAMESPACE=camera rosrun image_proc image_proc

