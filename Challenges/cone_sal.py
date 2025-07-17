"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_f.py

Title: Lab F - Line Follower

Author: Team 4

Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR should automatically identify the color of a line it sees, then drive on the
center of the line throughout the obstacle course. The RACECAR should also identify
color changes, following colors with higher priority than others.
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Constants
MIN_CONTOUR_AREA = 80
CROP_FLOOR = ((10, 0), (480, rc.camera.get_width()))

#GREEN = ((27, 84, 184), (90, 250, 255))
ORANGE = ((1, 1, 1), (7, 225, 225))
BLUE = ((98, 126, 1), (104, 255, 255))

# Variables
speed = 0.0
angle = 0.0
contour_center = None
contour_area = 0
color_name = None
kp = 0.0005
kd = 0.00001  # Derivative gain
last_angle = 0
last_error = 0
setpoint = 0
time = 0
########################################################################################
# Functions
########################################################################################

def update_contour():
    global contour_center, contour_area, color_name, kp

    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
        return

    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

    blue_contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
    blue = rc_utils.get_largest_contour(blue_contours, MIN_CONTOUR_AREA)
    blue_area = rc_utils.get_contour_area(blue) if blue is not None else 0

    orange_contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])
    orange = rc_utils.get_largest_contour(orange_contours, MIN_CONTOUR_AREA)
    orange_area = rc_utils.get_contour_area(orange) if orange is not None else 0

    if blue_area > orange_area and blue_area > 0:
        contour_center = rc_utils.get_contour_center(blue)
        contour_area = blue_area
        rc_utils.draw_contour(image, blue)
        rc_utils.draw_circle(image, contour_center)
        color_name = "blue"
    elif orange_area > 0:
        contour_center = rc_utils.get_contour_center(orange)
        contour_area = orange_area
        rc_utils.draw_contour(image, orange)
        rc_utils.draw_circle(image, contour_center)
        color_name = "orange"
    else:
        contour_center = None
        contour_area = 0

    rc.display.show_color_image(image)

def start():
    global speed, angle

    speed = 0
    angle = 0
    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)

    print(
        ">> Lab 2A - Color Image Line Following\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area"
    )

def update():
    global speed, angle, last_angle, kd, last_error, setpoint, time
    scan = rc.lidar.get_samples()
    right_distance = rc_utils.get_lidar_average_distance(scan, 90, 5)
    left_distance = rc_utils.get_lidar_average_distance(scan, -90, 5)
    front = rc_utils.get_lidar_average_distance(scan, 0, 2)
    print("Front", front)
    update_contour()

    if contour_center is not None and front < 1010:
        print(front)
        if color_name == "blue":
            print("blue")
            setpoint = 3*(rc.camera.get_width() / 4)
            if right_distance > 25:
                angle = -0.8
        elif color_name == "orange":
            print("orange")
            setpoint = rc.camera.get_width()
            if left_distance > 25:
                angle = 0.8
            
        
        present_value = contour_center[1]
        error = -(setpoint - present_value)

        dt = rc.get_delta_time()
        time += dt
        if dt == 0:
            dt = 1e-3

        derivative = (error - last_error) / time
        last_error = error

        angle = -(kp * error + kd * derivative)
        angle = rc_utils.clamp(angle, -1, 1)
        print(f"[PD] Error: {error}, Derivative: {derivative:.2f}, Angle: {angle:.2f}")

        last_angle = -angle
    else:
        print("Lost contour — holding last angle")
        angle = -(last_angle * 0.9)  # mild decay

    speed = 0.67
    rc.drive.set_speed_angle(speed, angle)

    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)

def update_slow():
    if rc.camera.get_color_image() is None:
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
