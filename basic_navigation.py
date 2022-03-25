from tello import Tello
import sys
from datetime import datetime
import time
# from PIL import Image
import cv2 
import numpy as np
import rospy
import std_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import camera_info_manager as cim
from sensor_msgs.msg import CameraInfo
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

class TelloVisualOdometry():
    def __init__(self):
        # Parameters
        self.loop_rate = rospy.Rate(30)
        self.drone_speed = int(100) #Speed of the Tello drone in cm/s (10-100)
        self.drone_speed = np.clip(self.drone_speed, 10, 100)
        
        # Set waypoints
        self.final_goal_reached = False
        self.goal_index = 0
        self.relative_waypoints = []
        half_box_len = 0.5 # Half the length of the box in meterss
        # Start with Drone placed at the center. 
        self.relative_waypoints.append([half_box_len, half_box_len, 0]) # Center to Top Left 
        self.relative_waypoints.append([-2*half_box_len, 0, 0]) # Top Left to Bottom Left
        self.relative_waypoints.append([0, -2*half_box_len, 0]) # Bottom Left to Bottom Right
        self.relative_waypoints.append([2*half_box_len, 0, 0]) # Bottom right to Top Right

        self.relative_waypoints.append([0, 2*half_box_len, 0]) # Top Right to Top Left
        self.relative_waypoints.append([-2*half_box_len, 0, 0]) # Top Left to Bottom Left
        self.relative_waypoints.append([0, -2*half_box_len, 0]) # Bottom Left to Bottom Right
        self.relative_waypoints.append([2*half_box_len, 0, 0]) # Bottom right to Top Right
        
        # self.relative_waypoints.append([-half_box_len, half_box_len, 0]) # Top Left to Center

        # Connect to Tello
        self.command_lock = False
        self.tello = Tello('', 8889)  
        self.send_command_until_ack('command')
        self.send_command_until_ack('streamon')

    def check_battery(self):
        while self.command_lock:
            rospy.sleep(0.1)   
        print("Checking battery percentage.")     
        if not self.command_lock:
            self.command_lock = True

            max_n_retry = 10
            n_retry = 0
            battery_percentage = -1
            while n_retry < max_n_retry:
                response = self.tello.send_command("battery?")
                try:
                    battery_percentage = int(response)
                    print("Battery at {} percent.".format(battery_percentage))
                    self.command_lock = False
                    return battery_percentage
                except:
                    print("Error: Could not convert response = {} to battery precentage.".format(response))
                
                rospy.sleep(0.1)
                n_retry = n_retry + 1
            
            self.command_lock = False

            if n_retry == max_n_retry:
                raise ValueError('Error: Max number of retries reached.')

    def send_command_until_ack(self, cmd_str):
        if not self.command_lock:
            self.command_lock = True

            ack = "not ok"
            max_n_retry = 10
            n_retry = 0
            while ack != 'ok' and n_retry < max_n_retry:
                ack = self.tello.send_command(cmd_str)
                print(ack)
                rospy.sleep(0.1)
                n_retry = n_retry + 1
            
            self.command_lock = False

            if n_retry == max_n_retry:
                raise ValueError('Error: Max number of retries reached.')     


    def start(self):
        # Takeoff 
        self.send_command_until_ack('takeoff')
        self.is_drone_in_the_air = True
        rospy.sleep(5)

        # Set Tello speed. 
        self.send_command_until_ack('speed ' + str(self.drone_speed))

        for i in range(len(self.relative_waypoints)):
            # Extract translations (convert from m to cm)
            x_cm = int(np.round(self.relative_waypoints[i][0]*100))
            y_cm = int(np.round(self.relative_waypoints[i][1]*100))
            z_cm = int(np.round(self.relative_waypoints[i][2]*100))

            x_cm = np.clip(x_cm, -500, 500)
            y_cm = np.clip(y_cm, -500, 500)
            z_cm = np.clip(z_cm, -500, 500)

            print("Moving to x={}, y={}, z={} relative to the current position. ".format(x_cm, y_cm, z_cm))
            self.send_command_until_ack('go {} {} {} {}'.format(x_cm, y_cm, z_cm, self.drone_speed))

            d = np.sqrt(x_cm**2 + y_cm**2 + z_cm**2)
            time_to_move = d / self.drone_speed
            rospy.sleep(time_to_move + 5.0)

        self.final_goal_reached = True
        self.end()


    def end(self):
        # Stop drone video
        self.tello.video_freeze()

        if self.final_goal_reached:
            print("\n\n\nFinal goal reached!!! Landing drone. =)\n\n\n")

        # Land the drone.
        self.send_command_until_ack('land') 
        rospy.sleep(5)
        print("Drone has landed.")

        # Print final battery percentage.
        self.check_battery()

    def __del__(self):
        print("TelloVisualOdometry object deconstructor called.")
        self.end()

        # Delete tello object to close socket. 
        del self.tello
        print("tello object deconstructed")

if __name__ == "__main__":
    rospy.init_node("Tello_Visual_Odometry_Node", anonymous=False)
    tello_node = TelloVisualOdometry()
    tello_node.start()
    del tello_node 