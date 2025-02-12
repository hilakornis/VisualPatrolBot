#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/TurboPi/')
import cv2
import time
import math
import signal
import Camera
import threading
import numpy as np
import yaml_handle
import HiwonderSDK.PID as PID
import HiwonderSDK.Misc as Misc
import HiwonderSDK.mecanum as mecanum

# color tracking
board = None
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
car = mecanum.MecanumChassis()

servo1 = 1500
servo2 = 1500
servo_x = servo2
servo_y = servo1

color_radius = 0
color_center_x = -1
color_center_y = -1

car_en = False
wheel_en = False
size = (640, 480)
target_color = ()
__isRunning = False

car_x_pid = PID.PID(P=0.15, I=0.001, D=0.0001) # pid initialization
car_y_pid = PID.PID(P=1.00, I=0.001, D=0.0001)
servo_x_pid = PID.PID(P=0.06, I=0.0003, D=0.0006)  
servo_y_pid = PID.PID(P=0.06, I=0.0003, D=0.0006)

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)


# Initial Position
def initMove():
    board.pwm_servo_set_position(1, [[1, servo1], [2, servo2]])

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# Turn off the motor
def car_stop():
    car.set_velocity(0,90,0)  # 关闭所有电机

#Set the RGB light color of the expansion board to match the color you want to track
def set_rgb(color):
    if color == "red":
        board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
    elif color == "green":
        board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
    elif color == "blue":
        board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
    else:
        board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

# Variable reset
def reset():
    global target_color, car_en
    global servo1, servo2, wheel_en
    global servo_x, servo_y, color_radius
    global color_center_x, color_center_y
    
    car_en = False
    wheel_en = False
    servo1 = servo_data['servo1']
    servo2 = servo_data['servo2']
    servo_x = servo2
    servo_y = servo1
    target_color = ()
    car_x_pid.clear()
    car_y_pid.clear()
    servo_x_pid.clear()
    servo_y_pid.clear()
    color_radius = 0
    color_center_x = -1
    color_center_y = -1
    

# app initialization call
def init():
    print("ColorTracking Init")
    load_config()
    reset()
    initMove()

# App starts playing method call
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")

# app stops playing method call
def stop():
    global __isRunning
    reset()
    initMove()
    car_stop()
    __isRunning = False
    set_rgb('None')
    print("ColorTracking Stop")

# App exit gameplay call
def exit():
    global __isRunning
    reset()
    initMove()
    car_stop()
    __isRunning = False
    set_rgb('None')
    print("ColorTracking Exit")

# Set the detection color
def setTargetColor(color):
    global target_color

    print("COLOR", color)
    target_color = color
    return (True, ())

# Setting up vehicle following
def setVehicleFollowing(state):
    global wheel_en
    
    print("wheel_en", state)
    wheel_en = state
    if not wheel_en:
        car_stop()
    return (True, ())

# Find the contour with the largest area
# The parameter is a list of contours to be compared.
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    areaMaxContour = None
    for c in contours:  # Go through all the contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # The contour with the largest area is valid only when the area is greater than 300 to filter out interference
                areaMaxContour = c
    return areaMaxContour, contour_area_max  # Returns the largest contour

# Robot movement logic processing
def move():
    print("Moving")
    global __isRunning, car_en, wheel_en
    global servo_x, servo_y, color_radius
    global color_center_x, color_center_y
    
    img_w, img_h = size[0], size[1]
    
    while True:
        if __isRunning:
            if color_center_x != -1 and color_center_y != -1:
                # Camera PTZ tracking
                # Tracking based on the camera's X-axis coordinates
                if abs(color_center_x - img_w/2.0) < 15: # If the movement is small, no movement is required.
                    color_center_x = img_w/2.0
                servo_x_pid.SetPoint = img_w/2.0    # set up
                servo_x_pid.update(color_center_x)  # current
                servo_x += int(servo_x_pid.output)  # Get PID output value
                
                servo_x = 800 if servo_x < 800 else servo_x  # Set the servo range
                servo_x = 2200 if servo_x > 2200 else servo_x
                
                # Tracking based on the camera's Y-axis coordinate
                if abs(color_center_y - img_h/2.0) < 10: # If the movement is small, no movement is required.
                    color_center_y = img_h/2.0
                servo_y_pid.SetPoint = img_h/2.0   # set up
                servo_y_pid.update(color_center_y) # current
                servo_y -= int(servo_y_pid.output) # Get PID output value
                
                servo_y = 1200 if servo_y < 1200 else servo_y # Set the servo range
                servo_y = 1900 if servo_y > 1900 else servo_y
                
                board.pwm_servo_set_position(0.02, [[1, servo_y], [2, servo_x]]) # Set the servo movement
                time.sleep(0.01)
                
                # Car body tracking
                if wheel_en:
                    # Tracking at near and far distances based on target size
                    if abs(color_radius - 100) < 10: 
                        car_y_pid.SetPoint = color_radius
                    else:
                        car_y_pid.SetPoint = 100
                    car_y_pid.update(color_radius)
                    dy = car_y_pid.output   # Get PID output value
                    dy = 0 if abs(dy) < 15 else dy # Setting the speed range
                    
                    # Tracking based on the X-axis servo value
                    if abs(servo_x - servo2) < 15:
                        car_x_pid.SetPoint = servo_x
                    else:
                        car_x_pid.SetPoint = servo2
                    car_x_pid.update(servo_x)
                    dx = car_x_pid.output   # Get PID output value
                    dx = 0 if abs(dx) < 15 else dx # Setting the speed range
                    
                    car.translation(dx, dy) # Set robot movement (X-axis speed, Y-axis speed)
                    car_en = True
                
                time.sleep(0.01)
                
            else:
                if car_en:
                    car_stop()
                    car_en = False
        else:
            if car_en:
                car_stop()
                car_en = False
            time.sleep(0.01)

# Running child threads
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# Robotic Image Processing
def run(img):
    global __isRunning, color_radius
    global color_center_x, color_center_y
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning:   # Check whether the gameplay is turned on, if not, return to the original image
        return img
     
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert the image to LAB space
    
    area_max = 0
    areaMaxContour = 0
    for i in target_color:
        if i in lab_data:
            frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))  #Perform bitwise operations on the original image and mask 
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # Open Operation
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # Closing operation
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Find the contour
            areaMaxContour, area_max = getAreaMaxContour(contours)  # Find the largest contour
    if area_max > 1000:  # The maximum area has been found
        (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)  # Get the minimum circumscribed circle
        color_radius = int(Misc.map(radius, 0, size[0], 0, img_w))
        color_center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
        color_center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
        if color_radius > 300:
            color_radius = 0
            color_center_x = -1
            color_center_y = -1
            return img
        
        cv2.circle(img, (color_center_x, color_center_y), color_radius, range_rgb[i], 2)
        
    else:
        color_radius = 0
        color_center_x = -1
        color_center_y = -1
            
    return img


#Close processing
def manual_stop(signum, frame):
    global __isRunning
    
    print('close...')
    __isRunning = False
    car_stop()  # Turn off all motors
    initMove()  # The steering wheel returns to its initial position

if __name__ == '__main__':
    import HiwonderSDK.ros_robot_controller_sdk as rrc
    print('running... hilak')
    board = rrc.Board()
    init()
    start()
    target_color = ('red',)
    camera = Camera.Camera()
    camera.camera_open(correction=True) # Enable correction, default is not enabled
    signal.signal(signal.SIGINT, manual_stop)
    while __isRunning:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240)) # Screen zoom to 320*240
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                print('Exit')
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
