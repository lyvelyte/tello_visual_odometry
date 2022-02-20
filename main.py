from tello import Tello
import sys
from datetime import datetime
import time
from PIL import Image
from PIL import ImageTk
import cv2 
import numpy as np

if __name__ == "__main__":
    tello = Tello('', 8889)  
    tello.send_command('command')
    tello.send_command('streamon')
    tello.send_command('takeoff') 
    
    time.sleep(10)

    # frame = tello.read()

    n_azimuth = 4
    r = 50  # Standoff distance (cm)
    dTheta = (2*np.pi)/n_azimuth
    dx = r-r*np.cos(dTheta)
    dy = r*np.sin(dTheta)
    dz = 0
    speed = 50

    # tello.send_command("left " + str(int(round(r))))  
    # time.sleep(5)
    # tello.send_command("cw " + str(int(round(np.rad2deg(np.arctan2(0.5, 2))))))  
    # time.sleep(5)

    frame = tello.read()
    frame_num = 8
    if frame is None or frame.size == 0:
        print("No image available.")
    else:
        print("Image received!")
        filename = "output/" + str(frame_num) + ".png"
        cv2.imwrite(filename, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        print("Saved " + filename)

    # tello.send_command("back " + str(int(round(r))))  
    # time.sleep(5)
  
    # print("Starting main tello loop.")
    # run_time = 1
    # start_time = time.time()
    # frame_num = 0
    # while(frame_num < n_azimuth):
    #     print("Frame Number = " + str(frame_num))
    #     print(dx, dy, np.rad2deg(dTheta))
    #     tello.send_command("right " + str(int(round(dy)))) 
    #     time.sleep(5)
    #     tello.send_command("forward " + str(int(round(dx))))  
    #     time.sleep(5)
    #     tello.send_command("ccw " + str(int(round(np.rad2deg(dTheta))))) 
    #     time.sleep(5)

    #     frame = tello.read()
    #     if frame is None or frame.size == 0:
    #         continue
    #     else:
    #         print("Image received!")
    #         filename = "output/" + str(frame_num) + ".png"
    #         cv2.imwrite(filename, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    #         print("Saved " + filename)
    #     frame_num = frame_num + 1

    # time.sleep(3)
    tello.send_command('land') 
    print("Done.")