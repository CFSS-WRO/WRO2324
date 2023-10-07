import numpy as np
import cv2
import time
import pyrealsense2 as rs
from realsense_depth import *
from jetracer.nvidia_racecar import NvidiaRacecar
car = NvidiaRacecar()
# rct {0, 1, 2} -- 0 : front  1 : right  2 : left

dc = DepthCamera()
bline,gline = [0,0,0,0],[0,0,0,0]
def show_xy(event, x, y, flags, param):
    	global point
    	if event == cv2.EVENT_LBUTTONDOWN:
    		point = (x,y)
    		print( point)
    		cv2.circle(color_frame, point, 4, (0, 0, 255))
    		#cv2.putText(color_frame, "{}".format(point), rctl(rct), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
cv2.namedWindow('color_frame')
cv2.setMouseCallback('color_frame',show_xy)
rct = 0
rctl = ["NULL", "RIGHT", "LEFT"]

def pl():
	cv2.putText(color_frame, "{}".format(rctl[rct], bline , gline), (30, 30),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
def sw():
	ret, depth_frame, color_frame = dc.get_frame()
	key = cv2.waitKey(1)
def show():
	cv2.imshow("bedges", bedges)
	cv2.imshow("gedges", gedges)
	cv2.imshow("color_frame", color_frame)
	pl()
while True:
	while rct == 0:
		if rct == 0:
			ret, depth_frame, color_frame = dc.get_frame()
			key = cv2.waitKey(1)
	
			# blue
			hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)
			# Set range for blue color and define mask, input: [Hue, Sat, Intensity].
			low_bluel = np.array([90, 150, 150])
			up_bluel = np.array([130, 255, 255])
			bmask = cv2.inRange(hsv, low_bluel, up_bluel)
			bedges = cv2.Canny(bmask, 75, 150)
			blines = cv2.HoughLinesP(bedges, 1, np.pi/180, 50, 50, 5)
			if blines is not None:
				bline = blines[0][0]
				#x1, y1, x2, y2 = blines[]
				if bline[1] < bline[3] :rct = 1
				else:rct = -1
		    
		    
			# green
			# Set range for green color and define mask, input: [Hue, Sat, Intensity].
			low_greenl = np.array([45, 150, 150])
			up_greenl = np.array([75, 255, 255])
			gmask = cv2.inRange(hsv, low_greenl, up_greenl)
			gedges = cv2.Canny(gmask, 75, 150)
			glines = cv2.HoughLinesP(gedges, 1, np.pi/180, 50, 50, 10)
			#if glines is not None:
			#	line = glines[0][0]
				#x1, y1, x2, y2 = blines[]
			#	if gline[1] > gline[3]:
			#		rct = 1
			#	else:rct = 2
		
			#print(bline , gline)
			#cv2.putText(color_frame, "{}".format(rctl[rct]), (30, 30),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
			show()
		if rct != 0:
			#cv2.imshow("bmask", bmask)
			show()
			car.steering = 0
			car.throttle = 0
			break 
		if key == 27:
			car.steering = 0
			car.throttle = 0
			cv2.destroyAllWindows()
			break
		
	if rct == 0:
		break
	ret, depth_frame, color_frame = dc.get_frame()
	key = cv2.waitKey(1)
	show()
	car.throttle = 0
	car.steering = rct
	print(car.steering, car.throttle, rctl[rct], bline , gline)
	if key == 27:
		car.steering = 0
		car.throttle = 0
		cv2.destroyAllWindows()
		break	
