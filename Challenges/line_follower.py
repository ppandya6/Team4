# TODO: Implement line_follower

=======

=======
"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_f.py

Title: Lab F - Line Follower

Author: Matthew Liong

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
MIN_CONTOUR_AREA = 30
CROP_FLOOR = ((250, 0), (480, rc.camera.get_width()))

GREEN = ((27, 84, 184), (90, 250, 255))
ORANGE = ((5, 16, 195), (12, 225, 225))

# Variables
speed = 0.0
angle = 0.0
contour_center = None
contour_area = 0
color_name = None
kp = 0.007
kd = 0.0015  # Derivative gain
last_angle = 0
last_error = 0

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

    green_contours = rc_utils.find_contours(image, GREEN[0], GREEN[1])
    green = rc_utils.get_largest_contour(green_contours, MIN_CONTOUR_AREA)
    green_area = rc_utils.get_contour_area(green) if green is not None else 0

    orange_contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])
    orange = rc_utils.get_largest_contour(orange_contours, MIN_CONTOUR_AREA)
    orange_area = rc_utils.get_contour_area(orange) if orange is not None else 0

    if green_area > orange_area and green_area > 0:
        contour_center = rc_utils.get_contour_center(green)
        contour_area = green_area
        rc_utils.draw_contour(image, green)
        rc_utils.draw_circle(image, contour_center)
        color_name = "green"
        kp = 0.0045
    elif orange_area > 0:
        contour_center = rc_utils.get_contour_center(orange)
        contour_area = orange_area
        rc_utils.draw_contour(image, orange)
        rc_utils.draw_circle(image, contour_center)
        color_name = "orange"
        kp = 0.003
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
    global speed, angle, last_angle, kd, last_error

    update_contour()

    if contour_center is not None:
        setpoint = rc.camera.get_width() // 2
        present_value = contour_center[1]
        error = setpoint - present_value

        dt = rc.get_delta_time()
        if dt == 0:
            dt = 1e-3

        derivative = (error - last_error) / dt
        last_error = error

        angle = -(kp * error + kd * derivative)
        angle = rc_utils.clamp(angle, -1, 1)
        print(f"[PD] Error: {error}, Derivative: {derivative:.2f}, Angle: {angle:.2f}")

        last_angle = angle
    else:
        print("Lost contour — holding last angle")
        angle = last_angle * 0.9  # mild decay

    speed = 0.9
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


