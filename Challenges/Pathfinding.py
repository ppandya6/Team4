#team 2's code, i (matthew) cleaned it up, removed a few redundant bits, changed up variable names, added all the comments
import sys
import math
import numpy as np

sys.path.insert(0, '../library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()
speed = 0.0
angle = 0.0

def fix_angle(deg):
    return deg + 360 if deg < 0 else deg 
    #if degrees is negative then add 360 so its positive but still equal
    #eg -225=45

def start():
    global speed, angle
    rc.drive.set_max_speed(0.5) #for the sim
    speed = 1.0
    angle = 0.0
    rc.set_update_slow_time(0.5)

def update():
    global speed, angle

    scan_data = rc.lidar.get_samples()

    #fov_span = 120
    window_size = 30 #width that the car checks around a given heading (window size, centered around angle value)
    min_dist = 120 #far distances are only checked if they exceed this value
    step = 2 #goes through angles between two bounds, this is the step

    best_opening = 0 #resets best_opening after every loop -> the highest (closest distance) for all checked paths
    chosen_heading = 0 #resets chosen_heading after every loop -> the angle of best_opening

    for heading in range(-75, 75, step): #steps between -75 and 75 by 2 to find angles (create angle windows)
        start = heading - window_size // 2 #start value of for loop (heading - half of width)
        end = heading + window_size // 2 #end value of for loop (heading + half of width)
        #result: checks a window of given size around the given heading

        samples = [] #clears/resets samples
        for angle in range(start, end + 1): #checks all angles in the given window, finds the distance at that angle (end+1 because range is not inclusive)
            adjusted_angle = fix_angle(angle) #prevents negative angles
            dist = rc_utils.get_lidar_average_distance(scan_data, adjusted_angle) #gets distance at an angle within the window
            if dist is not None and dist > min_dist: #only append if distance exists and is above a certain value -> saves computing time/power
                samples.append(dist) #appends the far distance into samples

        if not samples: #if samples is empty = no distances above min_dist
            #if there are no distances above the minimum distance threshold, stop checking that angle and move on
            continue 

        candidate_clearance = min(samples) #takes the closest point within samples -> the most imminently dangerous point     
        if candidate_clearance > best_opening: #takes the farthest closest point -> makes it the least risky path because there are the fewest/farthest obstructions at that angle
            chosen_heading = heading #if it is the safest path, save the heading
            best_opening = candidate_clearance #if it is the safest path, then save that as the threshold (only change it if there is a better path)

    forward_angle = 60 #angle to find distance
    sample_window = 2 #window size
    #average distance at 60 degrees left/right w/ 2 degree window (we checked this angle for wall follow)
    kp = 0.003 

    _, closest_right = rc_utils.get_lidar_closest_point(scan_data, (0, 180)) #closest point on the right half
    _, closest_left = rc_utils.get_lidar_closest_point(scan_data, (180, 360)) #closest point on the left half
    right_forward = rc_utils.get_lidar_average_distance(scan_data, forward_angle, sample_window) #average distance at 60 degrees left w/ 2 degree window
    left_forward = rc_utils.get_lidar_average_distance(scan_data, 360 - forward_angle, sample_window) #average distance at 60 degrees left w/ 2 degree window
    
    #basically creates a triangle
    #closest_right/closest_left finds closest point to the wall -> guarantees a 90 degree angle will be drawn between the LIDAR and the closest wall, even if the wall is slanted
    #right_forward/left_forward finds the distance at an angle (forward_angle) with given window size
    #since closest_right/left forms a right angle, and right_forward/left_forward is opposite, then we know it is the hypoteneuse
    #we can find wall_length using the pythagorean theorem (it is a leg, we have the other leg and the hypoteneuse)
    
    #b = sqrt(a**2 - c**2)
    right_wall_length = math.sqrt(max(0, right_forward ** 2 - closest_right** 2)) #pythagorean theorem (avoids complex numbers with max())
    left_wall_length = math.sqrt(max(0, left_forward ** 2 - closest_left ** 2)) #pythagorean theorem (avoids complex numbers with max())

    #if the wall length gets bigger, then the wall is either closer or slanting towards the car

    error = right_wall_length - left_wall_length #if error = 0, then the right and left wall lengths are equal, meaning the car is centered
    wall_adjust = rc_utils.clamp(error * kp, -1, 1) #clamps

    merged_angle = rc_utils.clamp((chosen_heading / 75 + wall_adjust) / 2.0, -1.0, 1.0) 
    #we checked heading between -75 and 75. dividing by 75 normalizes it between -1 and 1. 
    #wall angle is already clamped between -1 and 1
    #takes the average between these two values to calculate angle

    speed_threshold=400 #if best_opening is above this distance, set speed up. else, lower speed
    speed = 1.0 if best_opening > speed_threshold else 0.6 #if the path forward is very clear (the closest point is 220+ cm away), set speed = 1. if it is not so clear, set speed = 0.6

    print(best_opening)
    rc.drive.set_speed_angle(speed, merged_angle) 

def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
