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
        self.save_images = False
        self.loop_rate = rospy.Rate(30)

        # CV Bridge
        self.image = None
        self.br = CvBridge()

        # Load Camera Calibration Info
        self.caminfo = cim.loadCalibrationFile('calibration.yaml', 'camera')
        
        # Publishers
        self.pub_img = rospy.Publisher('camera/image_raw', Image, queue_size=1)
        self.pub_caminfo = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=1, latch=True)
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        # Connect to Tello
        self.tello = Tello('', 8889)  
        self.tello.send_command('command')
        self.tello.send_command('streamon')

    def start(self):
        # # Takeoff 
        # self.tello.send_command('takeoff')
        # time.sleep(5)

        prev_frame_hash = hash(str("A_Blank_Starting_String"))

        # Main Loop 
        frame_num = 0
        while (not rospy.is_shutdown()):
            # Attempt to read image from Tello's Camera
            frame = self.tello.read()
            
            new_frame_available = False

            cur_frame_hash = hash(str(frame))
            if prev_frame_hash != cur_frame_hash:
                prev_frame_hash = cur_frame_hash
                new_frame_available = True
            

            if frame is None or frame.size == 0:
                print("Failed to read tello image, no image was available.")
            elif new_frame_available:
                cur_timestamp = rospy.Time.now()

                # Construct and publish image to ROS. 
                image_msg = self.br.cv2_to_imgmsg(frame, encoding='rgb8')
                image_msg.header.frame_id = "tello_camera"
                image_msg.header.stamp = cur_timestamp
                self.pub_img.publish(image_msg)
                print("Image read from tello and published to ROS.")
              
                # Construct and publish camera info (with calibration data) to ROS. 
                self.caminfo.header.frame_id = "tello_camera"
                self.caminfo.header.stamp = cur_timestamp
                self.pub_caminfo.publish(self.caminfo)

                # Construct and Publish the fixed tf message to transform from the tello's camera to the drone
                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = "tello_drone"
                t.child_frame_id = "tello_camera"
                t.header.stamp = cur_timestamp
                t.transform.translation.x = 0.03
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                rot_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
                # rot_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
                t.transform.rotation.x = rot_quat[0]
                t.transform.rotation.y = rot_quat[1]
                t.transform.rotation.z = rot_quat[2]
                t.transform.rotation.w = rot_quat[3]
                tf_msg = tf2_msgs.msg.TFMessage([t])
                self.pub_tf.publish(tf_msg)

                # Construct and Publish the fixed tf message to transform from the ar_marker to the ar_marker's orientation in the world
                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = "ar_marker"
                t.child_frame_id = "ar_marker_world_orientation"
                t.header.stamp = cur_timestamp
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                rot_quat = tf.transformations.quaternion_from_euler(-np.pi/2, -np.pi/2, 0)
                t.transform.rotation.x = rot_quat[0]
                t.transform.rotation.y = rot_quat[1]
                t.transform.rotation.z = rot_quat[2]
                t.transform.rotation.w = rot_quat[3]
                tf_msg = tf2_msgs.msg.TFMessage([t])
                self.pub_tf.publish(tf_msg)

                # Construct and Publish the fixed tf message to transform from the ar_marker to the World
                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = "ar_marker_world_orientation"
                t.child_frame_id = "world"
                t.header.stamp = cur_timestamp
                t.transform.translation.x = 1.5
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                rot_quat = tf.transformations.quaternion_from_euler(0, 0, np.pi)
                t.transform.rotation.x = rot_quat[0]
                t.transform.rotation.y = rot_quat[1]
                t.transform.rotation.z = rot_quat[2]
                t.transform.rotation.w = rot_quat[3]
                tf_msg = tf2_msgs.msg.TFMessage([t])
                self.pub_tf.publish(tf_msg)

                # Optionally save images to disk. 
                if self.save_images:                
                    filename = "../output_images/" + str(frame_num) + ".png"
                    cv2.imwrite(filename, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                    print("Saved " + filename)
                    frame_num = frame_num + 1

                self.loop_rate.sleep()
        else:
            print("No new frame available. Power on and reconnect to the tello network.")

        
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
    rospy.init_node("Tello_Visual_Odometry_Node", anonymous=False)
    tello_node = TelloVisualOdometry()
    tello_node.start()
    del tello_node 