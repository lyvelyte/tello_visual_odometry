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
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

class TelloVisualOdometry():
    def __init__(self):
        # Parameters
        self.save_images = False
        self.loop_rate = rospy.Rate(30)

        # CV Bridge
        self.image = None
        self.br = CvBridge()
        
        # Publishers
        self.pub = rospy.Publisher('camera/image_raw', Image, queue_size=1)
        self.caminfo = cim.loadCalibrationFile('calibration.yaml', 'camera')
        self.pub_caminfo = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=1, latch=True)    

        self.caminfo.header.frame_id = "tello_camera"
        self.caminfo.header.stamp = rospy.Time.now()
        self.pub_caminfo.publish(self.caminfo)

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        # Setup Tello
        self.tello = Tello('', 8889)  
        self.tello.send_command('command')
        self.tello.send_command('streamon')

    def start(self):
        # # Takeoff 
        # self.tello.send_command('takeoff')
        # time.sleep(5)

        # Loop for capturing images and publishing them to ROS. 
        frame_num = 0
        # while (not rospy.is_shutdown()) and (frame_num < 30*10):
        while (not rospy.is_shutdown()):
            frame = self.tello.read()

            if frame is None or frame.size == 0:
                print("Failed to read tello image, no image was available.")
            else:
                print("Image read from tello, publishing to ROS.")

                converted_image = Image()
                converted_image = self.br.cv2_to_imgmsg(frame, encoding='rgb8')
              
                frame_id = "tello_camera"

                now = rospy.Time.now()
                self.caminfo.header.stamp = now
                self.caminfo.header.frame_id = frame_id
                converted_image.header.frame_id = frame_id
                converted_image.header.stamp = now

                self.pub.publish(converted_image)
                self.pub_caminfo.publish(self.caminfo)

                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = frame_id
                t.header.stamp = now
                t.child_frame_id = "tello_camera_child"
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
  
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 0.0
  
                tfm = tf2_msgs.msg.TFMessage([t])
                self.pub_tf.publish(tfm)

                if self.save_images:                
                    filename = "../output_images/" + str(frame_num) + ".png"
                    cv2.imwrite(filename, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                    print("Saved " + filename)

            frame_num = frame_num + 1

            self.loop_rate.sleep()
        
    def __del__(self):
        print("TelloVisualOdometry object deconstructor called.")
        # Stop drone video
        self.tello.video_freeze()

        # # Land the drone.
        # time.sleep(3) 
        # self.tello.send_command('land') 
        # time.sleep(5)s
        # print("Drone has landed.")

        # Delete tello object to close socket. 
        del self.tello
        print("tello objet deconstructed")

if __name__ == "__main__":
    rospy.init_node("Tello_Visual_Odometry_Node", anonymous=True)
    tello_node = TelloVisualOdometry()
    tello_node.start()
    del tello_node 