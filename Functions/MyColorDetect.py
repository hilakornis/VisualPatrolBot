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

# color recognition
board = None
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

servo1 = 1500
servo2 = 1500
target_color = ('red', 'green', 'blue','purple')

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

# initial position
def initMove():
    board.pwm_servo_set_position(1, [[1, servo1], [2, servo2]])

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
    'purple': (128,0,128),
}

_stop = False
color_list = []
size = (640, 480)
__isRunning = False
detect_color = 'None'
start_pick_up = False
draw_color = range_rgb["black"]

# variable reset
def reset(): 
    global _stop
    global color_list
    global detect_color
    global start_pick_up
    global servo1, servo2
    
    _stop = False
    color_list = []
    detect_color = 'None'
    start_pick_up = False
    servo1 = servo_data['servo1']
    servo2 = servo_data['servo2']

# app initialization call
def init():
    print("ColorDetect Init")
    load_config()
    reset()
    initMove()

# app starts gameplay call
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorDetect Start")

# app stops gameplay call
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorDetect Stop")

# app exit gameplay call
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorDetect Exit")

def setTargetColor(color):
    global target_color

    target_color = color
    return (True, ())


#Set the RGB light color of the expansion board to match the color you want to track
def set_rgb(color):
    if color == "red":
        board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
    elif color == "green":
        board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
    elif color == "blue":
        board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
    elif color == "purple":
        board.set_rgb([[1,128,0,128],[2,128,0,128]])
    else:
        board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

# Find the contour with the largest area
# The argument is a list of contours to compare
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # Go through all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # The maximum area contour is only valid when the area is greater than 300 to filter interference
                area_max_contour = c

    return area_max_contour, contour_area_max  # Returns the largest contour

# Robot movement logic processing
def move():
    global _stop
    global __isRunning
    global detect_color
    global start_pick_up
    

    while True:
        if __isRunning:
            if detect_color != 'None' and start_pick_up:  # Color patch detected
                board.set_buzzer(1900, 0.1, 0.9, 1)# Set the buzzer to sound for 0.1 seconds
                set_rgb(detect_color) # Set the colored lights on the expansion board to be the same as the detected color
                
                if detect_color == 'red' :  # Red detected, nod
                    for i in range(0,3):
                        board.pwm_servo_set_position(0.3, [[1, servo1-100]])
                        time.sleep(0.3)
                        board.pwm_servo_set_position(0.3, [[1, servo1+100]])
                        time.sleep(0.3)
                    board.pwm_servo_set_position(0.5, [[1, servo1]])  # Return to initial position
                    time.sleep(0.5)       
                    
                else:                      # If green or blue is detected, shake the head
                    for i in range(0,3):
                        board.pwm_servo_set_position(0.35, [[2, servo2-150]])
                        time.sleep(0.35)
                        board.pwm_servo_set_position(0.35, [[2, servo2+150]])
                        time.sleep(0.35)
                    board.pwm_servo_set_position(0.5, [[2, servo2]])  # Return to initial position
                    time.sleep(0.5)
                    
                _stop = True
                detect_color = 'None'
                start_pick_up = False
                set_rgb(detect_color)
                
            else:
                time.sleep(0.01)
        else:
            if _stop:
                initMove()  # Return to initial position
                _stop = False
                time.sleep(1.5)               
            time.sleep(0.01)

# Run child thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# Robot image processing
def run(img):
    global __isRunning
    global start_pick_up
    global detect_color, draw_color, color_list
    
    if not __isRunning:  # Check whether the gameplay is enabled. If not enabled, return to the original image.
        return img
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB space

    color_area_max = None
    max_area = 0
    areaMaxContour_max = 0
    if not start_pick_up:
        for i in target_color:
            if i in lab_data:
                frame_mask = cv2.inRange(frame_lab,
                                             (lab_data[i]['min'][0],
                                              lab_data[i]['min'][1],
                                              lab_data[i]['min'][2]),
                                             (lab_data[i]['max'][0],
                                              lab_data[i]['max'][1],
                                              lab_data[i]['max'][2]))  #Perform bit operations on the original image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # Open operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # closed operation
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the outline
                areaMaxContour, area_max = getAreaMaxContour(contours)  # Find the maximum contour
                if areaMaxContour is not None:
                    if area_max > max_area:  # Find the largest area
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 2500:  # Found the largest area
            rect = cv2.minAreaRect(areaMaxContour_max)
            box = np.intp(cv2.boxPoints(rect))
            
            cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)
            if not start_pick_up:
                if color_area_max == 'red':  # Red is the largest
                    color = 1
                elif color_area_max == 'green':  # Green is the biggest
                    color = 2
                elif color_area_max == 'blue':  # blue is the biggest
                    color = 3
                else:
                    color = 0
                color_list.append(color)
                if len(color_list) == 3:  # multiple judgments
                    # average
                    color = np.mean(np.array(color_list))
                    color_list = []
                    start_pick_up = True
                    if color == 1:
                        detect_color = 'red'
                        draw_color = range_rgb["red"]
                    elif color == 2:
                        detect_color = 'green'
                        draw_color = range_rgb["green"]
                    elif color == 3:
                        detect_color = 'blue'
                        draw_color = range_rgb["blue"]
                    else:
                        start_pick_up = False
                        detect_color = 'None'
                        draw_color = range_rgb["black"]
        else:
            if not start_pick_up:
                detect_color = 'None'
                draw_color = range_rgb["black"]
        
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2) # 把检测到的颜色打印在画面上
    
    return img


#Processing before closing
def manual_stop(signum, frame):
    global __isRunning
    
    print('Closed...')
    __isRunning = False
    initMove()  # The servo returns to the initial position


if __name__ == '__main__':
    import HiwonderSDK.ros_robot_controller_sdk as rrc
    board = rrc.Board()
    init()
    start()
    camera = Camera.Camera()
    camera.camera_open(correction=True) # Enable distortion correction, not enabled by default
    signal.signal(signal.SIGINT, manual_stop)
    while __isRunning:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240)) # The screen is zoomed to 320*240
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()

