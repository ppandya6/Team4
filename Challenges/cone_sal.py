"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: Lab H - Cone Slalom 

Title: Student 

Author: Karen Huang 

Purpose: The goal of this lab is to program your RACECAR to navigate through a "cone slalom" course, which is where your vehicle must swerve between red and blue cones to reach the finish line. 

Expected Outcome: Rules of the Road: To navigate through the cone slalom course, program your RACECAR to follow two rules:

Drive on the right side of RED cones
Drive on the left side of BLUE cones
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import numpy as np
import math

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
# >> Constants
# The smallest contour we will recognize as a valid contour (Adjust threshold!)
MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((280, 0), (rc.camera.get_height(), rc.camera.get_width()))
#CROP_FLOOR = ((100, 0), (rc.camera.get_height(), rc.camera.get_width()))

# HSV Thresholds
BLUE = ((90, 115, 115),(120, 255, 255), "BLUE")
RED = ((170, 115, 115),(10, 255, 255), "RED")
COLORS = [RED, BLUE] # List of colors

MIN_RANGE = 5   # Ignore values < 5 cm (noise)
MAX_RANGE = 250 # Ignore anything farther than 2.5 meter

Kp = -2

# Variables about turning
TURN_THRESHOLD = 70 # 100 cm

# >> Variables
cone_distance = 0
closest_cone_center = None  # The (pixel row, pixel column) of contour
closest_cone_area = 0  # The area of contour
speed = 0
angle = 0

cones = []
queue = [] # The queue of instructions
is_queue_empty = "YES"  # The queue of instructions
closest_cone_color = "None" # The current color of the cone
state = "SEARCHING"



########################################################################################
# Functions
########################################################################################

# [FUNCTION] Find the colors in the image
def find_cones(image):
    #color_name = "None" # The detected color from the list of color thresholds
    #color_area = 0 # The area of the detected color
    #color_center = None # The center of the detected color
    local_cones = []

    for (hsv_lower, hsv_upper, color) in COLORS:
        contours = rc_utils.find_contours(image, hsv_lower, hsv_upper)
        largest_contour = rc_utils.get_largest_contour(contours)
        if largest_contour is not None:
            local_contour_center = rc_utils.get_contour_center(largest_contour)
            local_contour_area = rc_utils.get_contour_area(largest_contour)
            if local_contour_area > MIN_CONTOUR_AREA:
                cone_area = local_contour_area
                cone_center = local_contour_center
                cone_color = color
                local_cones.append({"cone_color": cone_color, "cone_area": cone_area, "cone_center": cone_center})

    return sorted(local_cones, key=lambda c: c["cone_area"], reverse=True)  # Sorts all found cones so that the largest one comes first

def find_closest_cone(lidar):
    global cone_distance

    #Angles in front of the car (e.g. 270° to 360° to 90°)
    front_angles = list(range(270, 360)) + list(range(0, 91))

    front_distances = [lidar[a % 360] for a in front_angles if lidar[a % 360] > 0.0]

    valid_distances = [d for d in front_distances if 5 < d < 1000]

    if valid_distances:
        cone_distance = min(valid_distances)
        #print(f"cone_distance = {cone_distance}")

# [FUNCTION] Finds contours in the current color image and uses them to update
# contour_center and contour_area
def update_contour():
    global cones


    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0


    else:
        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        cones = find_cones(image)


# [FUNCTION] Appends the correct instructions to make a 90 degree right turn to the queue
def turnRight():
    global queue

    queue.clear()

    queue.append([1.2, 0.5, 1])
    queue.append([0.5, 0.5, 0.5])
    queue.append([0.2, 0.5, 0])
    queue.append([1.5, 0.5, -1])
    queue.append([0.1, 0.5, 0])

# [FUNCTION] Appends the correct instructions to make a 90 degree left turn to the queue
def turnLeft():
    global queue

    queue.clear()

    queue.append([1.2, 0.5, -1])
    queue.append([0.5, 0.5, -0.5])
    queue.append([0.2, 0.5, 0])
    queue.append([1.5, 0.5, 1])
    queue.append([0.1, 0.5, 0])


# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle

    # Initialize variables
    speed = 0.5 
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    #rc.set_update_slow_time(0.5)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global cones
    global queue
    global speed
    global angle
    global closest_cone_color
    global closest_cone_center
    global closest_cone_area
    global is_queue_empty
    global state
    global cone_distance

    # Search for contours in the current color image
    update_contour()

    lidar = rc.lidar.get_samples()
    find_closest_cone(lidar) 

    if cone_distance is None:
        return

    if closest_cone_center is None:
        closest_cone_center = 0

    print(f"Gcone_distance = {cone_distance}") 
    if cones:
        closest_cone = cones[0]   # Largest cone
        closest_cone_color = closest_cone["cone_color"]
        closest_cone_center = closest_cone["cone_center"]

    if cone_distance > MAX_RANGE:
        state = "SEARCHING"
        angle = 0
        speed = 0.5 
    elif TURN_THRESHOLD <= cone_distance <= MAX_RANGE:
        state = "APPROACHING"
        if closest_cone_center is not None:
            setpoint = rc.camera.get_width() // 2  # x=320

            present_value = closest_cone_center[1]

            # Calculate the error signal e(t)
            error = setpoint - present_value

            # Calculate the control signal u(t)
            angle = Kp * error

            # Clamp angle to prevent assertion error
            angle = rc_utils.clamp(angle, -1, 1)
    else:
        state = "TURNING"
        if is_queue_empty == "YES":
            if closest_cone_color == "RED":
                turnRight()
                is_queue_empty = "NO"
            elif closest_cone_color == "BLUE":
                turnLeft()
                is_queue_empty = "NO"

        elif is_queue_empty == "NO":
            if len(queue) > 0:
                speed = queue[0][1]
                angle = queue[0][2]
                queue[0][0] -= rc.get_delta_time()
                if queue[0][0] <= 0:
                    queue.pop(0)

            elif len(queue) == 0:
                angle = 0
                is_queue_empty = "YES"
                state = "SEARCHING"


    # Send speed and angle commands to the RACECAR
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

