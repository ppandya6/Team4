# TODO: Implement grand_prix
# Line follower is prioritized over Pathfinding. Pitch is used in order to determine ramp is present or ntot to maintain pathfinding stability.
import sys
import cv2 as cv
import math
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

########################################################################################
# Global Variables
########################################################################################

# Contour detection constants
MIN_CONTOUR_AREA = 30
VISIBLE_HEIGHT = 80
BOTTOM = 480
TOP = BOTTOM - VISIBLE_HEIGHT
CROP_FLOOR = ((TOP, 0), (BOTTOM, rc.camera.get_width()))

# Color thresholds
RED1 = ((0, 100, 100), (10, 255, 255))
RED2 = ((160, 100, 100), (179, 255, 255))
GREEN = ((27, 84, 184), (90, 250, 255))
BLUE = ((100, 150, 100), (130, 255, 255))

# State variables
contour_center = None
contour_area = 0
color_name = "none"
kp = 0.015
kd = 0.00025
last_error = 0
last_angle = 0
speed = 0.0
angle = 0.0
elapsed_time = 0
position = []
time = []

########################################################################################
# Vision: Color Contour Detection
########################################################################################

def update_contour():
    global contour_center, contour_area, color_name, kp

    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
        color_name = "none"
        return

    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

    red_contours1 = rc_utils.find_contours(image, RED1[0], RED1[1])
    red_contours2 = rc_utils.find_contours(image, RED2[0], RED2[1])
    red_all = red_contours1 + red_contours2
    red = rc_utils.get_largest_contour(red_all, MIN_CONTOUR_AREA)
    red_area = rc_utils.get_contour_area(red) if red is not None else 0

    green_contours = rc_utils.find_contours(image, GREEN[0], GREEN[1])
    green = rc_utils.get_largest_contour(green_contours, MIN_CONTOUR_AREA)
    green_area = rc_utils.get_contour_area(green) if green is not None else 0

    blue_contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
    blue = rc_utils.get_largest_contour(blue_contours, MIN_CONTOUR_AREA)
    blue_area = rc_utils.get_contour_area(blue) if blue is not None else 0

    if red_area > 0 and red_area >= green_area and red_area >= blue_area:
        contour_center = rc_utils.get_contour_center(red)
        contour_area = red_area
        rc_utils.draw_contour(image, red)
        rc_utils.draw_circle(image, contour_center)
        color_name = "red"
        kp = 0.6
    elif green_area > 0 and green_area >= blue_area:
        contour_center = rc_utils.get_contour_center(green)
        contour_area = green_area
        rc_utils.draw_contour(image, green)
        rc_utils.draw_circle(image, contour_center)
        color_name = "green"
        kp = 0.001
    elif blue_area > 0:
        contour_center = rc_utils.get_contour_center(blue)
        contour_area = blue_area
        rc_utils.draw_contour(image, blue)
        rc_utils.draw_circle(image, contour_center)
        color_name = "blue"
        kp = 0.01
    else:
        contour_center = None
        contour_area = 0
        color_name = "none"

########################################################################################
# LiDAR Pathfinding
########################################################################################

def lidar_pathfind():
    global angle, speed
    scan_data = rc.lidar.get_samples()
    window_size = 30
    min_dist = 120
    step = 2
    pitch_deg = rc.physics.get_attitude()[1]  # Get pitch angle
    best_opening = 0
    chosen_heading = 0

    for heading in range(-75, 75, step):
        start = heading - window_size // 2
        end = heading + window_size // 2
        samples = []
        for angle_check in range(start, end + 1):
            adjusted_angle = (angle_check + 360) % 360
            dist = rc_utils.get_lidar_average_distance(scan_data, adjusted_angle)
            if dist is not None and dist > min_dist:
                samples.append(dist)
        if not samples:
            continue
        candidate_clearance = min(samples)
        if candidate_clearance > best_opening:
            chosen_heading = heading
            best_opening = candidate_clearance

    forward_angle = 60
    sample_window = 2
    kp_wall = 0.003

    _, closest_right = rc_utils.get_lidar_closest_point(scan_data, (0, 180))
    _, closest_left = rc_utils.get_lidar_closest_point(scan_data, (180, 360))
    right_forward = rc_utils.get_lidar_average_distance(scan_data, forward_angle, sample_window)
    left_forward = rc_utils.get_lidar_average_distance(scan_data, 360 - forward_angle, sample_window)

    right_wall_length = math.sqrt(max(0, right_forward ** 2 - closest_right**2))
    left_wall_length = math.sqrt(max(0, left_forward ** 2 - closest_left**2))

    wall_error = right_wall_length - left_wall_length
    wall_adjust = rc_utils.clamp(wall_error * kp_wall, -1, 1)

    ### NEW: Pitch-aware logic to override navigation ###
    on_ramp = abs(pitch_deg) > 8  # Adjust threshold as needed (Degrees)
    if on_ramp:
        print("[IMU] On ramp detected — switching to wall-following only")
        heading_angle = wall_adjust  # Ignore chosen_heading
    else:
        heading_angle = rc_utils.clamp((chosen_heading / 75.0 + wall_adjust) / 2.0, -1.0, 1.0)

    speed = 1.0 if best_opening > 500 else 0.7
    angle = heading_angle


########################################################################################
# RACECAR Behavior Logic
########################################################################################

def start():
    global speed, angle
    speed = 0
    angle = 0
    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)
    print(">> Line Follower + Pathfinding (Fallback Mode Enabled)")

def update():
    global speed, angle, last_angle, last_error, elapsed_time, position, time

    update_contour()  # check for line
    
    if contour_center is not None:
        # Line following mode
        setpoint = rc.camera.get_width() // 2
        present_value = contour_center[1]
        error = setpoint - present_value
        dt = rc.get_delta_time() or 1e-3
        derivative = (error - last_error) / dt
        last_error = error

        angle = -(kp * error + kd * derivative)
        angle = rc_utils.clamp(angle, -1, 1)
        last_angle = angle

        speed = 1 if abs(angle) < 0.1 else rc_utils.clamp((angle ** 2) + 0.1, -1, 1)
        print(f"[LINE] Color: {color_name}, Error: {error}, Angle: {angle:.2f}")
    else:
        # Pathfinding fallback
        print("[LIDAR] No line found — switching to pathfinding mode")
        lidar_pathfind()

    rc.drive.set_speed_angle(speed, angle)

    # Logging and plotting
    elapsed_time += rc.get_delta_time()
    position.append(angle)
    time.append(elapsed_time)

    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)
    if rc.controller.is_down(rc.controller.Button.X):
        plt.figure()
        plt.plot(time, position, label="Angle")
        plt.axis((0, 20, -1, 1))
        plt.xlabel("Time (s)")
        plt.ylabel("Angle")
        plt.title("Angle over Time")
        plt.grid(True)
        plt.legend()
        plt.savefig("angle_plot.png")
        print("[INFO] Plot saved to 'angle_plot.png'.")

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
# Entry Point
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
