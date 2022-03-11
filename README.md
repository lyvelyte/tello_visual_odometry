# tello_visual_odometry
Navigation code for tello edu drone using visual odometry and the AR Toolkit

## Main Code Setup (This repo, using Anaconda):
0. Clone this repo and open a terminal in the repo directory. 
1. conda create -n tello 
2. conda activate tello
3. conda install pip
4. pip install -r requirements.txt
5. sudo apt install libswscale-dev libavcodec-dev libavutil-dev
6. cd h264decoder
7. pip install . 

## Catkin Workspace setup (Requires Ubunut 18.04 and ROS Melodic - Does NOT work in 20.04/Noetic)
1. source /opt/ros/melodic/setup.bash
2. mkdir -p ~/catkin_ws/src
3. cd ~/catkin_ws/
4. catkin_make 
5. cd src
6. git clone https://github.com/ar-tools/ar_tools
7. git clone https://github.com/ros-drivers/camera_umd
8. git clone https://github.com/ros-perception/camera_info_manager_py
9. sudo apt-get install libv4l-dev
10. sudo ln -s /usr/include/libv4l1-videodev.h   /usr/include/linux/videodev.h
11. cd ..
12. catkin_make

## Anaconda Setup (If not already instealled)
1. wget https://repo.anaconda.com/archive/Anaconda3-2021.11-Linux-x86_64.sh
2. bash Anaconda3-2021.11-Linux-x86_64.sh


## Camera Calibration:
1.  rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/camera/image_raw

## Running
1. roslaunch ar_pose ar_pose_tello.launch
2. python main.py

