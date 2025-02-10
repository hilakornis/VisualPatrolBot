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

# Integrated visual patrol + color detection
board = None
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# Initialize shared parameters
servo1 = 1500
servo2 = 1500
img_centerx = 320
line_centerx = -1
size = (640, 480)
target_color = ()
__isRunning = False
swerve_pid = PID.PID(P=0.001, I=0.00001, D=0.000001)
car = mecanum.MecanumChassis()


lab_data = None
servo_data = None
detect_color = 'None'
start_pick_up = False
color_confirmed = False
color_confidence = 0

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
    'purple': (255,255,114),
}

def load_config():
    global lab_data, servo_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

def initMove():
    car_stop()
    board.pwm_servo_set_position(1, [[1, servo1], [2, servo2]])

def reset():
    global line_centerx, target_color, servo1, servo2, color_confidence
    line_centerx = -1
    target_color = ()
    servo1 = servo_data['servo1']+350
    servo2 = servo_data['servo2']
    color_confidence = 0

def car_stop():
    car.set_velocity(0,90,0)

def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max and contour_area_temp > 300:
            contour_area_max = contour_area_temp
            area_max_contour = c
    return area_max_contour, contour_area_max


def color_detection(frame):
    global detect_color, color_confidence, color_confirmed
    if not target_color:
        return frame
    
    frame_gb = cv2.GaussianBlur(frame, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    
    max_area = 0
    detected = 'None'
    for color in target_color:
        if color not in lab_data:
            continue
        mask = cv2.inRange(frame_lab,
                          (lab_data[color]['min'][0],
                          lab_data[color]['min'][1],
                          lab_data[color]['min'][2]),
                          (lab_data[color]['max'][0],
                          lab_data[color]['max'][1],
                          lab_data[color]['max'][2]))
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        cnt, area = getAreaMaxContour(contours)
        if cnt is not None and area > max_area:
            max_area = area
            detected = color
    
    # Confidence Mechanism
    if detected == detect_color:
        color_confidence = min(color_confidence + 1, 5)
    else:
        color_confidence = max(color_confidence - 1, 0)
    
    if color_confidence >= 5 and not color_confirmed:
        color_confirmed = True
        detect_color = detected
        board.set_buzzer(1900, 0.5, 0, 1)
        set_rgb(detected)
        return True
    return False

def set_rgb(color):
    if color == "red":
        board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
    elif color == "green":
        board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
    elif color == "blue":
        board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
    else:
        board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

# Moving the thread of control
def patrol_move():
    global __isRunning, color_confirmed
    while True:
        if __isRunning and not color_confirmed:
            if line_centerx > 0:
                if abs(line_centerx-img_centerx) < 10:
                    line_centerx = img_centerx
                swerve_pid.SetPoint = img_centerx
                swerve_pid.update(line_centerx)
                angle = -swerve_pid.output
                car.set_velocity(50, 90, angle)
            else:
                car_stop()
        else:
            car_stop()
            time.sleep(0.01)

th = threading.Thread(target=patrol_move)
th.setDaemon(True)
th.start()

# Image processing main logic
def run(img):
    global line_centerx, __isRunning, color_confirmed
    if not __isRunning:
        return img
    
    if color_detection(img.copy()):
        __isRunning = False
        return img
    
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    
    
    return img

def start_patrol(color):
    global __isRunning, target_color, color_confirmed
    target_color = (color,)
    color_confirmed = False
    __isRunning = True
    print(f"Start patrolling for {color}")

def manual_stop(signum, frame):
    global __isRunning
    __isRunning = False
    car_stop()

if __name__ == '__main__':
    import HiwonderSDK.ros_robot_controller_sdk as rrc
    board = rrc.Board()
    load_config()
    initMove()
    camera = Camera.Camera()
    camera.camera_open(correction=True)
    signal.signal(signal.SIGINT, manual_stop)
    
    target = 'red'
    start_patrol(target)
    
    while True:
        img = camera.frame
        if img is not None:
            Frame = run(img)
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            if cv2.waitKey(1) == 27 or color_confirmed:
                break
        else:
            time.sleep(0.01)
    
    camera.camera_close()
    cv2.destroyAllWindows()
    print(f"Target color {target} found!")