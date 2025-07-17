"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: Lab H

Title: Cone Slalom

Author: Matthew Liong

Purpose: Navigate through a system of cones, going to the right of red cones and to the left of blue ones

Expected Outcome: Weave through the cones without collision
"""

########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import cv2 as cv
import time
########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

MIN_CONTOUR_AREA = 100


# Declare any global variables here
speed=0
angle=0

contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

BLUE = ((90, 100, 100), (120, 255, 255))  # The HSV range for the color blue
RED1 = ((0,50,50),(10,255,255))  #Lower red HSV range
RED2 = ((170,50,50),(180,255,255)) #Higher red HSV range

color=''

MIN_DISTANCE=175

state=0

turning=0

queue=[]

last_color=None

lost_start_time = None
scan_direction = 1  # 1 = turn right, -1 = turn left
########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    pass # Remove 'pass' and write your source code for the start() function here
def update_contour():
    global contour_area
    global contour_center
    global largest_contour_area 
    global largest_contour
    global speed
    global angle 
    global color

    largest_contour_area=0
    largest_contour=None

    color=''

    image=rc.camera.get_color_image()
    image=cv.cvtColor(image,cv.COLOR_BGR2HSV)
    image = rc_utils.crop(image, (180,0), (rc.camera.get_height(),rc.camera.get_width()))

    if image is None:
        contour_area=0
    else:
        red_mask1=cv.inRange(image,RED1[0],RED1[1])
        red_mask2=cv.inRange(image,RED2[0],RED2[1])
        red_mask=red_mask1 | red_mask2 #combines the two masks, for the upper and lower bounds of red HSV
        blue_mask=cv.inRange(image,BLUE[0],BLUE[1])

        
        
        red_contours,_=cv.findContours(red_mask,cv.RETR_LIST,cv.CHAIN_APPROX_SIMPLE)
        blue_contours,_=cv.findContours(blue_mask,cv.RETR_LIST,cv.CHAIN_APPROX_SIMPLE)

        for contour in red_contours:
                if cv.contourArea(contour)>largest_contour_area:
                    largest_contour_area=cv.contourArea(contour)
                    largest_contour=contour
                    if largest_contour_area>MIN_CONTOUR_AREA:
                        color='RED'

        for contour in blue_contours:
                if cv.contourArea(contour)>largest_contour_area:
                    largest_contour_area=cv.contourArea(contour)
                    largest_contour=contour
                    if largest_contour_area>MIN_CONTOUR_AREA:
                        color='BLUE'

    try:
        contour_center = rc_utils.get_contour_center(largest_contour)
        rc_utils.draw_circle(image,contour_center)
    except:
        pass
    rc.display.show_color_image(red_mask)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle
    global turning
    update_contour()
    speed=1
    scan = rc.lidar.get_samples()

    #closestDistance=float('inf')
    #closestAngle=None
    closestAngle, closestDistance = rc_utils.get_lidar_closest_point(scan)

    if color=='RED' and closestDistance<100:
        angle=1
        turning=1
    elif color=='RED' and closestDistance>250 and turning==1:
         angle=-0.5
         turning=0
    elif color=='RED' and turning==1:
         angle=0




    elif color=='BLUE' and closestAngle >= 60 and closestAngle >= 180:
        angle=-1
        turning=1
    elif color=='BLUE' and closestAngle <= 300 and closestAngle >= 180 and turning == 1:
         angle=0.5
         turning=0
    print(rc_utils.get_lidar_closest_point(scan,(-90,-10))[0])

    rc.drive.set_speed_angle(speed, angle)


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
