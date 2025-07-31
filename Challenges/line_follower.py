Line follower using path planning, becoming line-path follower:

import sys
import numpy as np
import cv2

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()
speed = 0.0
angle = 0.0
current_color = None
def start():
    global speed, angle
    rc.drive.set_max_speed(0.5)  # Set maximum speed
    speed = 0.9                  # Initial forward speed
    angle = 0.0                  # Initial steering angle
    rc.set_update_slow_time(0.5) # Set slow update timer (unused here)

# --- Define HSV thresholds for red, blue, green ---
def get_color_mask(hsv):
    # Red has two ranges in HSV due to wrapping around hue = 0
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    # Blue and green ranges
    lower_blue = np.array([100, 150, 100])
    upper_blue = np.array([130, 255, 255])
    lower_green = np.array([40, 100, 100])
    upper_green = np.array([80, 255, 255])

    # Priority 1: Red
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    if cv2.countNonZero(red_mask) > 300:
        current_color = "red"
        return red_mask  # Return red if strong signal

    # Priority 2: Green
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    if cv2.countNonZero(green_mask) > 300:
        current_color = "green"
        return green_mask

    # Priority 3: Blue
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    if cv2.countNonZero(blue_mask) > 300:
        current_color = "blue"
        return blue_mask

    

    return None  # No valid line found

def update():
    global speed, angle
    image = rc.camera.get_color_image()  # Get camera image

    if image is None:
        rc.drive.set_speed_angle(0, 0)
        return

    height, width, _ = image.shape  # Get dimensions of the image
    cropped = image[height // 4 :, :]  # Focus on bottom half of image

    hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)  # Convert to HSV

    mask = get_color_mask(hsv)  # Get mask of desired line color

    print(f"[DEBUG] Currently following: {current_color}")


    if mask is None:
        rc.drive.set_speed_angle(0.1, -1)  # Move slowly forward if no line
        return

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        rc.drive.set_speed_angle(0.1, -1)  # Still no line found
        return

    largest = max(contours, key=cv2.contourArea)  # Pick the biggest line contour
    filled_mask = np.zeros_like(mask)  # Make a blank mask
    cv2.drawContours(filled_mask, [largest], -1, 255, -1)  # Fill the contour onto mask

    # Windowed region scoring (like LiDAR heading search)
    num_regions = 15  # Divide image into 15 vertical regions
    region_width = width // num_regions
    region_scores = []

    for i in range(num_regions):
        x_start = i * region_width
        region = filled_mask[:, x_start : x_start + region_width]
        score = cv2.countNonZero(region)  # Count pixels in each region
        region_scores.append(score)

    best_idx = int(np.argmax(region_scores))  # Best heading = region with most line
    chosen_heading = (best_idx - num_regions // 2) / (num_regions // 2)  # Normalize [-1, 1]

    # Get contour center for center offset
    M = cv2.moments(largest)
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])  # Centroid x
        center_offset = (cx - width // 2) / (width // 2)  # Normalize [-1, 1]
    else:
        center_offset = 0.0

    kp = 0.5  # Centering sensitivity
    curve_adjust = rc_utils.clamp(center_offset * kp, -1.0, 1.0)  # Clamp offset

    # Combine region heading and center offset
    merged_angle = rc_utils.clamp((chosen_heading + curve_adjust) / 2.0, -1.0, 1.0)

    # Dynamic speed: more visible line = higher speed
    total_line_pixels = cv2.countNonZero(filled_mask)
    speed = 1.0 if total_line_pixels > 5000 else 0.6

    rc.drive.set_speed_angle(speed, merged_angle)  # Send drive command

def update_slow():
    pass  # No slow update logic needed

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()


# TODO: Implement line_follower

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
import matplotlib.pyplot as plt

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
kp = 0.015
#kd = 0.0015  # Derivative gain
#kd = 0.001 # --> 0.00295
kd=0.00025
#kd = 0.1 * Ku * Tu
#Kp=0.8*Ku 
#kp = 0.001
#ku = 0.00125
#Tu = 2
last_angle = 0
last_error = 0
time = []
position=[]
elapsed_time=0
########################################################################################
# Functions
########################################################################################
def speed_func(angle):
    return -(angle**2)
    

def update_contour():
    global contour_center, contour_area, color_name, kp, time

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
        kp = 0.001
    elif orange_area > 0:
        contour_center = rc_utils.get_contour_center(orange)
        contour_area = orange_area
        rc_utils.draw_contour(image, orange)
        rc_utils.draw_circle(image, contour_center)
        color_name = "orange"
        kp = 0.001
        
        
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
    global speed, angle, last_angle, kp, kd, last_error,elapsed_time, position, time

    update_contour()

    speed=1
    elapsed_time += rc.get_delta_time() 
    #print(elapsed_time)
    if elapsed_time < 22:
        print("<22s")
        speed=0.85
        kp = 0.0155
        kd = 0.01
    else:
        print('past')
        kd=0.00025

    
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

    

    
    rc.drive.set_speed_angle(speed, angle)

    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)

    elapsed_time += rc.get_delta_time()
    '''
    scan = rc.lidar.get_samples()
    right_distance = rc_utils.get_lidar_average_distance(scan, 90, 5)
    left_distance = rc_utils.get_lidar_average_distance(scan, -90, 5)
    error = right_distance - left_distance
    '''
    position.append(angle)
    time.append(elapsed_time)
    

    if rc.controller.is_down(rc.controller.Button.X):
        print("[INFO] Saving plot to 'angle_plot.png'...")
        plt.figure()
        plt.plot(time, position, label="Angle")
        plt.axis((0, 20, -1, 1))
        plt.xlabel("Time (s)")
        plt.ylabel("Angle")
        plt.title("Angle over Time")
        plt.grid(True)
        plt.legend()
        plt.savefig("angle_plot.png")
        print("[INFO] Plot saved to 'angle_plot.png'. You can view it later.")

    

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
