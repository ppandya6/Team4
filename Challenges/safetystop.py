"""
MIT BWSI Autonomous RACECAR
MIT License
bwsix RC101 - Fall 2023

File Name: template.py << [Modify with your own file name!]

Title: [PLACEHOLDER] << [Modify with your own title]

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: [PLACEHOLDER] << [Write the purpose of the script here]

Expected Outcome: [PLACEHOLDER] << [Write what you expect will happen when you run
the script.]
"""

########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(0, '../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here

front_distance=0
speed=0.0
angle=0.0

########################################################################################
# Functions
########################################################################################

def update_lidar():
    global front_distance
    scan = rc.lidar.get_samples()
    #print(scan)
    if scan.size == 0:
        front_distance = 999
    else:
        front_distance = rc_utils.get_lidar_average_distance(scan, 0, 4)
    if front_distance == 0:
        front_distance = 999
        
def start():
    pass # Remove 'pass' and write your source code for the start() function here

def update():
    global speed
    global angle
    global front_distance

    update_lidar()
    
    if front_distance < 350:
        # KP = -.05
        # setpoint = 350
        # error = setpoint - front_distance

        # speed = error*KP
        # print(speed)
        speed = -1

        if front_distance < 65:
            speed = 0
    else:
        (_, speed) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
        
    (angle, _) = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)
    
    speed = rc_utils.clamp(speed, -1, 1)
    #(_, speed) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    #(angle, _) = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)
    
    print(front_distance)    
    print("speed", speed)
    
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
