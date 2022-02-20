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
    tello.send_command('land') 