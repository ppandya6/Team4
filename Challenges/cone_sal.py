# TODO: Implement cone_sal
# lab_h/cone_slalom_lidar_state.py

import sys              # for path manipulation
import time        

sys.path.insert(1, '../../library') 
import racecar_core # Racecar API
import racecar_utils as rc_utils  # LIDAR helper
import cv2 as cv
import numpy as np

# ————————————————
# CONFIGURATION
# ————————————————
rc = racecar_core.create_racecar()
MIN_CONT = 30
CROP_FLOOR = ((0, 0), (rc.camera.get_height(), rc.camera.get_width()))
RED_LOW   = ((  0, 120,  70), ( 10, 255, 255))
RED_HIGH  = ((170, 120,  70), (180, 255, 255))
BLUE      = ((100, 150,   0), (140, 255, 255))
angle = 0
speed = 0.6
# ————————————————
# GLOBAL STATE
# ————————————————
rc = racecar_core.create_racecar()  # Racecar instance


# ————————————————
# LIFECYCLE FUNCTIONS
# ————————————————
def start():
    pass

# cone salom start
def update():
    global rc, angle, speed, MIN_CONT, CROP_FLOOR, RED_LOW, RED_HIGH, BLUE
    image = rc.camera.get_color_image()
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(hsv, RED_LOW[0], RED_LOW[1])
    mask2 = cv.inRange(hsv, RED_HIGH[0], RED_HIGH[1])
    red_mask = mask1 | mask2
    blue_mask = cv.inRange(hsv, BLUE[0], BLUE[1])

    reds   = rc_utils.find_contours(red_mask,  MIN_CONT)
    blues  = rc_utils.find_contours(blue_mask, MIN_CONT)

    # 4) pick your target (here: red first, then blue)
    if   reds:
        contours = reds
        color    = 'red'
    elif blues:
        contours = blues
        color    = 'blue'
    else:
        rc.drive.stop()
        return

    # 5) get the biggest contour
    c = max(contours, key=cv.contourArea)
    x,y,w,h = cv.boundingRect(c)
    cx = x + w/2
    img_w = rc.camera.get_width()

    # 6) compute errors
    steer_err = (cx - img_w/2) / (img_w/2)
    range_err = DESIRED_H - h

    # 7) P-control
    speed    = np.clip(K_SPEED * range_err, -1, 1)
    steering = np.clip(K_STEER * steer_err, -1, 1)

    # 8) you could even vary behavior by color
    #    e.g. slower approach for blue cones:
    if color == 'blue':
        speed *= 0.7

    # 9) drive!
    rc.drive.go(speed, steering)
    




def update_slow():
    pass

if __name__ == '__main__':
    rc.set_start_update(start, update, update_slow)
    rc.go()
