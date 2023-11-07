import numpy as np
import cv2
import time
from pynput import mouse
import pyrealsense2 as rs
from realsense_depth import *
# Initialize Camera Intel Realsense
dc = DepthCamera()
point1 = (200, 260)
point2 = (400, 260)

from jetracer.nvidia_racecar import NvidiaRacecar
car = NvidiaRacecar()

rFlag = False
gFlag = False

# Start a while loop
        while True:
            car.throttle = 0.2

            # Reading the video from the webcam in image frames
            ret, depth_frame, color_frame = dc.get_frame()

            # Convert the color_frame in BGR(RGB color space) to HSV(hue-saturation-value) color space
            hsvFrame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)

            # Set range for red color and define mask, input: [Hue, Sat, Intensity].
            red_lower = np.array([160, 140, 72], np.uint8)
            red_upper = np.array([190, 255, 255], np.uint8)
            red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    
            # Set range for green color and define mask
            green_lower = np.array([40, 52, 72], np.uint8)
            green_upper = np.array([75, 255, 215], np.uint8)
            green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
    
            # Morphological Transform, Dilation for each color and bitwise_and operator between image 
            # frame and mask determines to detect only that particular color
            kernal = np.ones((5, 5), "uint8")

            # For red color
            red_mask = cv2.dilate(red_mask, kernal)
            res_red = cv2.bitwise_and(color_frame, color_frame, mask=red_mask)

            # For green color
            green_mask = cv2.dilate(green_mask, kernal)
            res_green = cv2.bitwise_and(color_frame, color_frame, mask=green_mask)

            # Creating contour to track red color
            contours, hierarchy = cv2.findContours(
                red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )

            ret, depth_frame, color_frame = dc.get_frame()
            # Show distance for a specific point
            cv2.circle(color_frame, point1, 4, (255, 0, 255))
            cv2.circle(color_frame, point2, 4, (0, 255, 255))
            distance1 = depth_frame[point1[1], point1[0]]
            distance2 = depth_frame[point2[1], point2[0]]
            diff = distance2-distance1 #distance2 expected larger than distance1 for turning
            print("distance1 = ", distance1)
            print("distance2 = ", distance2)#, "\n")
            print("diff = ", diff, "\n")

            cv2.putText(color_frame, "{}mm".format(distance1), (point1[0], point1[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
            cv2.putText(color_frame, "{}mm".format(distance2), (point2[0], point2[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)


            for pic, contour in enumerate(contours):
                area_r = cv2.contourArea(contour)
                xr, yr, wr, hr = cv2.boundingRect(contour)
                if area_r > 8000 and wr <250: #Control in a correct area and avoid wrong recognition (large flat area which outside the box)
                    color_frame = cv2.rectangle(
                        color_frame, (xr, yr), (xr + wr, yr + hr), (0, 0, 255), 2
                    )
                    rFlag = True
                    if (xr >= 350):	
                        car.steering = 0.6
                        time.sleep(0.75)
                        car.steering = -0.6
                        time.sleep(0.75)
                        car.steering = 0
                        rFlag = False
        
                    else:
                        car.steering = 0.6
                        time.sleep(0.3)
                        car.steering = -0.6
                        time.sleep(0.3)
                        car.steering = 0
                        rFlag = False
    
                    cv2.putText(
                        color_frame,
                        "Red Colour",
                        (xr, yr),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 255),
                    )

            # Creating contour to track green color
            contours, hierarchy = cv2.findContours(
                green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
    
            for pic, contour in enumerate(contours):
                area_g = cv2.contourArea(contour)
                xg, yg, wg, hg = cv2.boundingRect(contour)
                if area_g > 8000 and wg < 250: #Control in a correct area and avoid wrong recognition (large flat area which outside the box)
                    color_frame = cv2.rectangle(
                        color_frame, (xg, yg), (xg + wg, yg + hg), (0, 255, 0), 2
                    )
                    gFlag = True
                    #print("area_g: ",area_g,"\n")
                    if (xg <= 350):
                        car.steering = -0.6
                        time.sleep(0.75)
                        car.steering = 0.6
                        time.sleep(0.75)
                        car.steering = 0
                        gFlag = False #Lower the flag after avoidance
    
                    else:
                        car.steering = -0.6
                        time.sleep(0.3)
                        car.steering = 0.6
                        time.sleep(0.3)
                        car.steering = 0
                        gFlag = False #Lower the flag after avoidance

                    cv2.putText(
                        color_frame,
                       "Green Colour",
                        (xg, yg),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 0),
                    )

            if (gFlag==False and rFlag==False): #If gFlag and rFlag are not raised
                if ((distance1 <= 580 or distance2 <= 580) and (distance1 != 0 and distance2 != 0)):
                    car.steering = 0.8
                else:
                    car.steering = 0

        # Program Termination
            cv2.imshow("Multiple Color Detection in Real-TIme", color_frame)
            cv2.imshow("G HSV", green_mask)
            cv2.imshow("R HSV", red_mask)
            key = cv2.waitKey(1)

            if key == 27:
                car.steering = 0
                car.throttle = 0
                cv2.destroyAllWindows()
                break
