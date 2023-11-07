import numpy as np
import cv2
import math
import time
from jetracer.nvidia_racecar import NvidiaRacecar
import pyrealsense2 as rs
from realsense_depth_Copy1copy import *
from inspect import currentframe

cf = currentframe()

car = NvidiaRacecar()
dc = DepthCamera()
#depth_scale = dc.get_scale()
# needed variables
bline,gline = [0,0,0,0],[0,0,0,0]

rct = 0 # rct {0, 1, 2} -- 0 : front  1 : left  2 : right
rctl = ["NULL", "LEFT", "RIGHT"]
end = False
imufirst = True
alpha = 0.98
last_ts_gyro = 0
totalgyroangleY = 0
dis_limit = 1100
cc = False
# funtions 
def get_lineinfo():
    
    return 

# imu data calculation
def imu():
    
    global imufirst, last_ts_gyro, accel_angle_y, totalgyroangleY
    #gather IMU data
    #accel = f[0].as_motion_frame().get_motion_data()
    #gyro = f[1].as_motion_frame().get_motion_data()
    #ts = f.get_timestamp()

    #calculation for the first frame
    if (imufirst):
        imufirst = False
        last_ts_gyro = ts
        # accelerometer calculation
        #accel_angle_z = math.degrees(math.atan2(accel.y, accel.z))
        #accel_angle_x = math.degrees(math.atan2(accel.x, math.sqrt(accel.y * accel.y + accel.z * accel.z)))
        #accel_angle_y = math.degrees(math.pi)
        accel_angle_y = 0
        return accel_angle_y
        
    #calculation for the second frame onwards

    # gyrometer calculations
    dt_gyro = (ts - last_ts_gyro) / 1000
    last_ts_gyro = ts

    #gyro_angle_x = gyro.x * dt_gyro
    gyro_angle_y = gyro.y * dt_gyro
    #gyro_angle_z = gyro.z * dt_gyro

    #dangleX = gyro_angle_x * 57.2958
    dangleY = gyro_angle_y * 57.2957795
    #dangleZ = gyro_angle_z * 57.2958

    #totalgyroangleX = accel_angle_x + dangleX
    #totalgyroangleY = accel_angle_y + dangleY
    totalgyroangleY = accel_angle_y + dangleY + totalgyroangleY
    #totalgyroangleZ = accel_angle_z + dangleZ

    #accelerometer calculation
    #accel_angle_z = math.degrees(math.atan2(accel.y, accel.z))
    #accel_angle_x = math.degrees(math.atan2(accel.x, math.sqrt(accel.y * accel.y + accel.z * accel.z)))
    # accel_angle_y = math.degrees(math.pi)
    #accel_angle_y = 0

    #combining gyrometer and accelerometer angles
    #combinedangleX = totalgyroangleX * alpha + accel_angle_x * (1-alpha)
    #combinedangleZ = totalgyroangleZ * alpha + accel_angle_z * (1-alpha)
    combinedangleY = totalgyroangleY #* (180/math.pi)

    
    #print("Angle -  X: " + str(round(combinedangleX,2)) + "   Y: " + str(round(combinedangleY,2)) + "   Z: " + str(round(combinedangleZ,2)))
    print(str(car.steering) +"   "+str(gyro.y)+"   "+ str(originalyaw) + "   Y: " + str(round(combinedangleY,2)))
    return combinedangleY

def show():
    cv2.putText(color_frame, "{}".format(rctl[rct], bline , gline), (30, 30),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
    cv2.imshow("bedges", bedges)
    cv2.imshow("color_frame", color_frame)


def bmask():
        global cuthsv
        low_bluel = np.array([100, 100, 100])
        up_bluel = np.array([145, 255, 255])
        bmask = cv2.inRange(hsv, low_bluel, up_bluel)
        return bmask
def gmask():
        low_gluel = np.array([100, 100, 100])
        up_gluel = np.array([145, 255, 255])
        gmask = cv2.inRange(hsv, low_bluel, up_bluel)
        return gmask
#def mask(self, hsv)
car.steering = 0
car.throttle = 0
#get in main loop
while True:
    # On start
    while rct == 0:

        # get frame and gyro data for turning detection and preset yaw
        ret, depth_frame, color_frame, accel, gyro, ts= dc.get_frame()
        #ret, accel, gyro, ts = dc.get_framec()

        if (imufirst):
            # set up first yaw
            accel_angle_y = imu()
            originalyaw = accel_angle_y
            ots = ts
            print(cf.f_lineno)
        else:
            yaw = imu()
            print(cf.f_lineno)   # for change of directing determine
        #roll, pitch, ya = [math.degrees(x) for x in rs.rs2_eulerian_angle_from_imu_data(accel, gyro)]
        #print(ya)
        key = cv2.waitKey(1)
        # blue
        hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)
        # Set range for blue color and define mask, input: [Hue, Sat, Intensity].
        cuthsv = hsv[240:480, 0:640]
        low_bluel = np.array([100, 100, 100])
        up_bluel = np.array([145, 255, 255])
        bmask = cv2.inRange(cuthsv, low_bluel, up_bluel)

        #bmask = cv2.inRange(cuthsv, low_bluel, up_bluel)
        bedges = cv2.Canny(bmask, 75, 150)
        blines = cv2.HoughLinesP(bedges, 1, np.pi/180, 50, 50, 5)
        print(cf.f_lineno)
        if blines is not None:
            bline = blines[0][0]
            #x1, y1, x2, y2 = blines[]
            #if bline[1] < bline[3] :rct = 1
            #else:rct = 2
            if bline[0] < bline[2] :
                if bline[1] < bline[3] :
                    rct = 2
                    print(cf.f_lineno)
                elif bline[1] > bline[3] :
                    rct = 1
                    print(cf.f_lineno)
        #print(bline , gline)
        #cv2.putText(color_frame, "{}".format(rctl[rct]), (30, 30),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        cv2.putText(color_frame, "{}".format(rctl[rct], bline , gline), (30, 30),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        cv2.imshow("bedges", bedges)
        cv2.imshow("color_frame", color_frame)
        if rct != 0:
            cv2.putText(color_frame, "{}".format(rctl[rct], bline , gline), (30, 30),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
            cv2.imshow("bedges", bedges)
            cv2.imshow("color_frame", color_frame)
            car.steering = 0
            car.throttle = 0
            print(get_lineinfo())
            
            break
        if ((ts-ots)/1000) >= 10:
            car.throttle = 0
            print(cf.f_lineno)
        if key == 27:
            end = True
            break
            
    if end:
        car.steering = 0
        car.throttle = 0
        cv2.destroyAllWindows()
        break
    #ret, depth_frame, color_frame = dc.get_framef()
    #ret, accel, gyro, ts = dc.get_framec()
    key = cv2.waitKey(1)	
    cv2.putText(color_frame, "{}".format(rctl[rct], bline , gline), (30, 30),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
    cv2.imshow("bedges", bedges)
    cv2.imshow("color_frame", color_frame)
    #depth_frame[point[1], point[0]]
    direct = imu()
    print(direct)
    car.throttle = 0
    car.steering = 0
    print(car.steering, car.throttle, rctl[rct], bline , gline)
    point = (320, 260)
    pointl = (0, 260)
    pointr = (638, 260)
    t = 0
    print(cf.f_lineno)
    break
if rct == 1:
    while True:
        ret, depth_frame, color_frame, accel, gyro, ts= dc.get_frame()
        checkdf = True
        direct = imu()
        # Show distance for a specific point
        #cv2.circle(color_frame, point, 4, (0, 0, 255))
        distance = depth_frame[point[1],point[0]] 
        disr = depth_frame[point[1],317]
        disl = depth_frame[point[1],312]
        distancel = depth_frame[pointl[1], pointl[0]]
        distancelm = depth_frame[pointl[1], 40]
        distancer = depth_frame[pointr[1], pointr[0]]
        distancerm = depth_frame[pointr[1], 600]
        print("distance l= ", distance, "\n", t, car.throttle, car.steering)
        cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        cv2.imshow("depth frame", depth_frame)
        cv2.imshow("Color frame", color_frame)
        key = cv2.waitKey(1)
        car.throttle = 0.2
        nowyawl = originalyaw - t*90
        if distance < dis_limit and distance != 0 and direct >= (nowyawl - 5) and direct <= (nowyawl + 5) :
            if disl <= disr + 1 and disr <= disl + 1:
                originalyaw =  t*90 - direct
            car.steering = 0.9
            while True:
                if checkdf:
                    if distancel < 350 and distancel !=0 and distancelm > distancel and direct > (nowyawl -2):
                        car.steering = 0.35
                        print(cf.f_lineno)
                        cc = True
                    elif cc:
                        #if distance > 600: 
                            car.steering = 0
                        #    print(cf.f_lineno)
                        #else: 
                            cc = False
                            print(cf.f_lineno)
                    else: 
                        car.steering =0.9
                        print(cf.f_lineno)
                #if distancer < 200:
                    #car.steering = -1
                if direct >= (originalyaw - (t+1)*90 - 3):
                    car.steering = 0
                    t += 1
                    print(cf.f_lineno)
                    break
                ret, accel, gyro, depth_frame, ts, checkdf = dc.get_imu()
                direct = imu()
                distancel = depth_frame[pointl[1], pointl[0]]
                distancelm = depth_frame[pointl[1], 40]
        else:
            print(cf.f_lineno)
            x = (direct +  t*90 - originalyaw) * (-1) /40
            if direct < (originalyaw - t*90 - 3):
                #x = (direct +  t*90 ) * (-1) /45
                if x >= 0.9:
                    print("loo")
                    x = 0.9
                car.steering = x
            elif direct > (originalyaw - t*90 + 3):
                if x < -1:
                    x = -1
                car.steering = x
            else:
                car.steering = 0
                if disl <= disr+1 and disr <= disl +1 and (:
                    originalyaw =  - t*90 - direct
        if key == 27:
            car.steering = 0
            car.throttle = 0
            break
        if key == 27:
            car.steering = 0
            car.throttle = 0
            break
if rct == 2:
    while True:
        ret, depth_frame, color_frame, accel, gyro, ts= dc.get_frame()
        checkdf = True
        direct = imu()
        # Show distance for a specific point
        cv2.circle(color_frame, point, 4, (0, 0, 255))
        cv2.circle(color_frame, pointl, 4, (0, 0, 255))
        cv2.circle(color_frame, pointr, 4, (0, 0, 255))
        distance = depth_frame[point[1],point[0]] 
        disr = depth_frame[point[1],317]
        disl = depth_frame[point[1],312]
        distancel = depth_frame[pointl[1], pointl[0]]
        distancer = depth_frame[pointr[1], pointr[0]]
        distancerm = depth_frame[pointr[1], 615]
        print("distance r= ", distance, "\n", t, car.throttle, car.steering)
        cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
        cv2.imshow("depth frame", depth_frame)
        cv2.imshow("Color frame", color_frame)
        key = cv2.waitKey(1)
        car.throttle = 0.2
        nowyawl = originalyaw + t*90
        if distance < dis_limit and distance != 0 and direct >= (nowyawl - 5) and direct <= (nowyawl + 5) :
            print(cf.f_lineno)
            if disl <= disr + 1 and disr <= disl + 1:
                originalyaw =  t*90 - direct
                print(cf.f_lineno)
            car.steering = 0.9
            #if direct < (originalyaw + (t+1)*90 - 3):
            while True:
                if checkdf:
                    if distancer < 350 and distancer !=0 and distancerm > distancer and direct > (nowyawl +2):
                        car.steering = -0.35
                        print(cf.f_lineno)
                        cc = True
                    elif cc:
                        #if distance > 600: 
                            car.steering = 0
                        #    print(cf.f_lineno)
                        #else: 
                            cc = False
                            print(cf.f_lineno)
                    else: 
                        car.steering =0.9
                        print(cf.f_lineno)
                #if distancer < 200:
                    #car.steering = -1
                if direct >= (originalyaw + (t+1)*90 - 3):
                    car.steering = 0
                    t += 1
                    print(cf.f_lineno)
                    break
                ret, accel, gyro, depth_frame, ts, checkdf = dc.get_imu()
                #ret,depth_frame,depth = dc.get_dframe()
                #if ret == False: continue
                direct = imu()
                distancer = depth_frame[pointr[1], pointr[0]]
                distancerm = depth_frame[pointr[1], 600]
        else:
            print(cf.f_lineno)
            x = (direct -  t*90 -originalyaw) * (-1) /40
            if direct < (originalyaw + t*90 - 3):
                #x = (direct +  t*90 ) * (-1) /45
                if x >= 0.9:
                    print("loo")
                    x = 0.9
                car.steering = x
            elif direct > (originalyaw + t*90 + 3):
                if x < -1:
                    x = -1
                car.steering = x
            else:
                car.steering = 0
                if disl <= disr+1 and disr <= disl +1:
                    originalyaw =  t*90 - direct
        if key == 27:
            car.steering = 0
            car.throttle = 0
            break
    if key == 27:
        car.steering = 0
        car.throttle = 0
        cv2.destroyAllWindows()
        end = True
