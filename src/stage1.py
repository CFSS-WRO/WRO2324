import numpy as np
from jetracer.nvidia_racecar import NvidiaRacecar
car = NvidiaRacecar()

import cv2
import pyrealsense2 as rs
from realsense_depth import *
# from pynput import mouse

import time
turn_time = 0.25

dis_limit = 610

point = (320, 260)

def show_distance(event, x, y, args, params):
    global point
    point = (x, y)
# Initialize Camera Intel Realsense
dc = DepthCamera()

def on_click(x,y, button, pressed):
    btn = button.name
    if btn == 'left':

# Create mouse event
#cv2.namedWindow("Color frame")
#cv2.setMouseCallback("Color frame", show_distance)

        while True:
            ret, depth_frame, color_frame = dc.get_frame()

        # Show distance for a specific point
            cv2.circle(color_frame, point, 4, (0, 0, 255))
            distance = depth_frame[point[1], point[0]]
            print("distance = ", distance, "\n")
    
            cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

            cv2.imshow("depth frame", depth_frame)
            cv2.imshow("Color frame", color_frame)
            key = cv2.waitKey(1)

           # car.throttle = 0.2

            if distance <= dis_limit and distance !=0:
             #   car.steering = -1.0
       # time.sleep(0.5)
		else:
       			car.steering = 0
    if btn == 'right':
        ret, depth_frame, color_frame = dc.get_frame()

        # Show distance for a specific point
        cv2.circle(color_frame, point, 4, (0, 0, 255))
        distance = depth_frame[point[1], point[0]]
        print("distance = ", distance, "\n")

        cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

        cv2.imshow("depth frame", depth_frame)
        cv2.imshow("Color frame", color_frame)
        key = cv2.waitKey(1)

       # car.throttle = 0.2

        if distance <= dis_limit and distance !=0:
          #  car.steering = -1.0
       # time.sleep(0.5)
        
        else:
            car.steering = 0



#with mouse.Listener(
#	on_click=on_click
#) as listener:
#	listener.join()
#if key == 27:
#	car.steering = 0
#	car.throttle = 0
	
#break

