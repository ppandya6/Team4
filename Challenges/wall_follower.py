# TODO: Implement wall_follower

=======
=======
"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_i.py

Title: Lab I - Wall Follower

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: This script provides the RACECAR with the ability to autonomously follow a wall.
The script should handle wall following for the right wall, the left wall, both walls, and
be flexible enough to handle very narrow and very wide walls as well.

Expected Outcome: When the user runs the script, the RACECAR should be fully autonomous
and drive without the assistance of the user. The RACECAR drives according to the following
rules:
- The RACECAR detects a wall using the LIDAR sensor a certain distance and angle away.
- Ideally, the RACECAR should be a set distance away from a wall, or if two walls are detected,
should be in the center of the walls.
- The RACECAR may have different states depending on if it sees only a right wall, only a 
left wall, or both walls.
- Both speed and angle parameters are variable and recalculated every frame. The speed and angle
values are sent once at the end of the update() function.

Note: This file consists of bare-bones skeleton code, which is the bare minimum to run a 
Python file in the RACECAR sim. Less help will be provided from here on out, since you've made
it this far. Good luck, and remember to contact an instructor if you have any questions!

Environment: Test your code using the level "Neo Labs > Lab I: Wall Follower".
Use the "TAB" key to advance from checkpoint to checkpoint to practice each section before
running through the race in "race mode" to do the full course. Lowest time wins!
"""

########################################################################################
# Imports
########################################################################################

import sys
import numpy as np

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here

speed=0
angle=0

kp = 0.006
kd = 0.005
prev_error = 0.0
prev_time = 0.0


########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    pass  # Remove 'pass' and write your source code for the start() function here


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle
    global kp, kd, prev_error, prev_time

    scan = rc.lidar.get_samples()
    right_distance = rc_utils.get_lidar_average_distance(scan, 45, 5)
    left_distance = rc_utils.get_lidar_average_distance(scan, -45, 5)
    print(right_distance)
    smallest_dist = np.minimum(left_distance, right_distance)
    if smallest_dist < 45:
        angle = 1
        speed = 0.7
        
    
    error = right_distance - left_distance

    # Time calculation
    current_time = rc.get_delta_time()
    if prev_time is None:
        prev_time = current_time

    dt = current_time - prev_time if current_time - prev_time > 0 else 1e-5

    # Derivative term
    derivative = (error - prev_error) / dt

    # PD Controller
    angle = (kp * error + kd * derivative)
    angle = rc_utils.clamp(angle, -1, 1)

    speed = 1

    rc.drive.set_speed_angle(speed, angle)

    # Update for next frame
    prev_error = error
    prev_time = current_time




# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass  # Remove pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()


"""

########################################################################################
# Imports
########################################################################################

import sys
import numpy as np

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here

speed=0
angle=0

kp = 0.007
kd = 0.00668
prev_error = 0.0
prev_time = 0.0
elapsed_time = 0

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    pass  # Remove 'pass' and write your source code for the start() function here


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle
    global kp, kd, prev_error, prev_time, elapsed_time

    scan = rc.lidar.get_samples()
    right_distance = rc_utils.get_lidar_average_distance(scan, 45, 5)
    left_distance = rc_utils.get_lidar_average_distance(scan, -45, 5)
    print("Right:", right_distance)
    print("Left:", left_distance)
    speed = 0.8
    elapsed_time += rc.get_delta_time() 
    print(elapsed_time)
    if elapsed_time > 10:
        speed = 1
        kp = 0.0067
        kd = 0.0070

    print("Speed", speed)
    current_time = rc.get_delta_time()
    if prev_time is None:
        prev_time = current_time
    dt = current_time - prev_time if current_time - prev_time > 0 else 1e-5

    error = right_distance - left_distance
    derivative = (error - prev_error) / dt
    
    small = np.minimum(left_distance, right_distance)
    if small > 35:
        angle = 0.5
    elif small != 35:
        angle = (kp * error + kd * derivative)
        print(angle)
        angle = rc_utils.clamp(angle, -1, 1)

    # Time calculation
    


    # Derivative term

    # PD Controller
    angle = (kp * error + kd * derivative)
    print(angle)
    angle = rc_utils.clamp(angle, -1, 1)
    
    

    rc.drive.set_speed_angle(speed, angle)

    # Update for next frame
    prev_error = error
    prev_time = current_time




# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass  # Remove pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()

"""

