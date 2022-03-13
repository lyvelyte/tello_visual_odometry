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
    def send_command_until_ack(self, cmd_str):
        ack = "not ok"
        max_n_retry = 20
        n_retry = 0
        while ack != 'ok' and n_retry < max_n_retry:
            ack = self.tello.send_command(cmd_str)
            print(ack)
            rospy.sleep(0.01)
            n_retry = n_retry + 1
        
        if n_retry == max_n_retry:
            raise ValueError('Error: Max number of retries reached.')

    def __init__(self):
        # Parameters
        self.camera_only = True
        self.loop_rate = rospy.Rate(30)
        self.drone_speed = int(100) #Speed of the Tello drone in cm/s (10-100)
        self.drone_speed = np.clip(self.drone_speed, 10, 100)
        
        # Set waypoints
        self.final_goal_reached = False
        self.goal_index = 0
        self.waypoints = []
        half_l = 0.3
        self.waypoints.append([0, 0, 0])
        self.waypoints.append([0, half_l, 0]) 
        self.waypoints.append([-half_l, half_l, 0])
        self.waypoints.append([-half_l, -half_l, 0])
        self.waypoints.append([half_l, -half_l, 0])
        self.waypoints.append([0, 0, 0])

        # CV Bridge
        self.image = None
        self.br = CvBridge()

        # Load Camera Calibration Info
        self.caminfo = cim.loadCalibrationFile('calibration.yaml', 'camera')
        
        # Publishers
        self.pub_img = rospy.Publisher('camera/image_raw', Image, queue_size=1)
        self.pub_caminfo = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=1, latch=True)
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        # Subscriber 
        self.tf_listener = tf.TransformListener()

        # Connect to Tello
        self.tello = Tello('', 8889)  
        self.send_command_until_ack('command')
        self.send_command_until_ack('streamon')

    def start(self):
        if not self.camera_only:
            # Takeoff 
            self.send_command_until_ack('takeoff')
            time.sleep(5)

            # Set Tello speed. 
            self.send_command_until_ack('speed ' + str(self.drone_speed))

        # Hash a dummy string to initialize the frame hash.
        prev_frame_hash = hash(str("A dummy string to initialize the frame hash."))
        prev_trans_hash = hash(str("A dummy string to initialize the frame hash."))

        # Main Loop 
        frame_num = 0
        while (not rospy.is_shutdown() and not self.final_goal_reached):
            goal_changed = False

            # Attempt to read image from Tello's Camera
            frame = self.tello.read()
            
            # Hash the frame to see if it's new or one we've already processed. 
            cur_frame_hash = hash(str(frame))
            new_frame_available = False
            if cur_frame_hash != prev_frame_hash:
                prev_frame_hash = cur_frame_hash
                new_frame_available = True

            if frame is None or frame.size == 0:
                print("Failed to read tello image, no image was available.")
            elif new_frame_available:
                # Get the current time. 
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
                rot_quat = tf.transformations.quaternion_from_euler(-np.pi/2, 0, -np.pi/2)
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
                t.transform.translation.x = 1.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                rot_quat = tf.transformations.quaternion_from_euler(0, 0, np.pi)
                t.transform.rotation.x = rot_quat[0]
                t.transform.rotation.y = rot_quat[1]
                t.transform.rotation.z = rot_quat[2]
                t.transform.rotation.w = rot_quat[3]
                tf_msg = tf2_msgs.msg.TFMessage([t])
                self.pub_tf.publish(tf_msg)

                # Construct and publish the varying tf message to transform from the world to the current goal. 
                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = "world"
                t.child_frame_id = "goal"
                t.header.stamp = cur_timestamp
                t.transform.translation.x = self.waypoints[self.goal_index][0]
                t.transform.translation.y = self.waypoints[self.goal_index][1]
                t.transform.translation.z = self.waypoints[self.goal_index][2]
                rot_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
                t.transform.rotation.x = rot_quat[0]
                t.transform.rotation.y = rot_quat[1]
                t.transform.rotation.z = rot_quat[2]
                t.transform.rotation.w = rot_quat[3]
                tf_msg = tf2_msgs.msg.TFMessage([t])
                self.pub_tf.publish(tf_msg)

                # Visual Navigation
                try:
                    if frame_num > 100:
                        # (trans,_) = self.tf_listener.lookupTransform("/tello_drone", "/world", rospy.Time(0))
                        (trans,_) = self.tf_listener.lookupTransform("/tello_drone", "/goal", rospy.Time(0))
                        (_,rot) = self.tf_listener.lookupTransform("/world", "/tello_drone", rospy.Time(0))
                        
                        cur_trans_hash = hash(str(trans) + str(rot))
                        new_transform_available = False
                        if prev_trans_hash != cur_trans_hash:
                            prev_trans_hash = cur_trans_hash
                            new_transform_available = True
                            print("\nFound new transform between /world and /tello_drone.")
                        else:
                            print("\nNo new transform between /world and /tello_drone found.")
                        
                        if new_transform_available:
                            # Extract translations (convert from m to cm)
                            x_cm = int(np.round(trans[0]*100))
                            y_cm = int(np.round(trans[1]*100))
                            z_cm = int(np.round(trans[2]*100))

                            x_cm = np.clip(x_cm, -500, 500)
                            y_cm = np.clip(y_cm, -500, 500)
                            z_cm = np.clip(z_cm, -500, 500)
                            # print("Translation: x={}, y={}, z={}".format(x_cm, y_cm, z_cm))
                            
                            # Extract rotations (convert to RPY, in degrees)
                            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
                            roll = int(np.round(np.rad2deg(roll)))
                            pitch = int(np.round(np.rad2deg(pitch)))
                            yaw = int(np.round(np.rad2deg(yaw)))
                            # print("Rotation: roll={}, pitch={}, yaw={}".format(roll, pitch, yaw))

                            max_t = max(abs(x_cm), abs(y_cm), abs(z_cm))
                            if (yaw < 10) and (max_t < 20):
                                print("Goal {} reached! Moving to next goal.".format(self.goal_index))
                                self.goal_index = self.goal_index + 1
                                goal_changed = True
                                frame_num = 0
                                if self.goal_index == len(self.waypoints):
                                    self.final_goal_reached = True

                            # Rotate first to align with world space.
                            if not self.camera_only and not self.final_goal_reached and not goal_changed:
                                # Tranlate drone
                                print("Adjusting translation, moving to x={}, y={}, z={} relative to the current position. ".format(x_cm, y_cm, z_cm))
                                self.send_command_until_ack('go {} {} {} {}'.format(x_cm, y_cm, z_cm, self.drone_speed))
                          
                                d = np.sqrt(x_cm**2 + y_cm**2 + z_cm**2)
                                time_to_move = d / self.drone_speed
                                rospy.sleep(time_to_move + 5.0)

                                # Rotate drone
                                if yaw > 0:
                                    print("Rotating drone clockwise {} degrees".format(abs(yaw)))
                                    self.send_command_until_ack('cw ' + str(abs(yaw)))
                                else:
                                    print("Rotating drone counter-clockwise {} degrees.".format(abs(yaw)))
                                    self.send_command_until_ack('ccw ' + str(abs(yaw)))
                                rospy.sleep(5.0)
                            
                except ValueError as err:
                    print(err.args)
                except:
                    print('Error: Transform between  /world and /tello_drone could not be looked up.')

                frame_num = frame_num + 1
                self.loop_rate.sleep()
            else:
                pass
                # print("No new frame available. Drone may not be powered on and connected.")
    
    def __del__(self):
        print("TelloVisualOdometry object deconstructor called.")
        # Stop drone video
        self.tello.video_freeze()

        if self.final_goal_reached:
            print("\n\n\nFinal goal reached!!! Landing drone. =)\n\n\n")

        # Land the drone.
        if not self.camera_only:
            time.sleep(3) 
            self.send_command_until_ack('land') 
            time.sleep(5)
            print("Drone has landed.")

        # Delete tello object to close socket. 
        del self.tello
        print("tello objet deconstructed")

if __name__ == "__main__":
    rospy.init_node("Tello_Visual_Odometry_Node", anonymous=False)
    tello_node = TelloVisualOdometry()
    tello_node.start()
    del tello_node 