#so this code was coded for the sim for obvious reasons, i am hoping it serves as a strong foundation
#team 2's base for pathfinding (wall follow function), i (matthew) cleaned it up, removed a few redundant bits, changed up variable names, added all the comments

#to-do
#tune hsv for: elevator, lines
#fix elevator code -> surroundings are different from sim
#test all that p control
#fix last state -> currently, i think it will fail on rainbow road -> it has to figure out some way to change states (maybe if a certain color exists??)

import sys
import math
import numpy as np
import cv2 as cv

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()
speed = 0.0
angle = 0.0
contour_center = None
contour_area = 0
color_name = None

GREEN = ((27, 84, 184), (90, 250, 255)) #the green values we tuned for the line follower quest
SIMGREEN = ((30,50,50),(80,255,255)) #green for the sim
BLUE = ((95, 150, 150), (120, 255, 255)) 
RED_1 = ((0, 50, 50), (10, 255, 255))      # low hue red
RED_2 = ((170, 50, 50), (180, 255, 255))   # high hue red
ORANGE = ((5, 16, 195), (12, 225, 225))

state=0 #initialize state machine

CROP_FLOOR = ((100, 100), (rc.camera.get_height(), rc.camera.get_width()-100)) 

MIN_CONTOUR_AREA = 30 

color = ''

contour_center = None
contour_area=0


#================================================================
def fix_angle(deg):
    return deg + 360 if deg < 0 else deg 
    #if degrees is negative then add 360 so its positive but still equal
    #eg -225=45

def update_contour():
    global contour_center, contour_area, color_name, color

    #reset area variables at the beginning of every update cycle
    red_area=0
    blue_area=0
    orange_area=0


    #set up image
    image = rc.camera.get_color_image()
    if image is None:
        contour_center = None
        contour_area = 0
        return

    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)


    #make masks
    blue_mask = cv.inRange(image,BLUE[0],BLUE[1])

    redmask1 = cv.inRange(image, RED_1[0], RED_1[1]) #lower hsv red
    redmask2 = cv.inRange(image, RED_2[0], RED_2[1]) #higher hsv red
    combined_red_mask = redmask1 | redmask2 #combine the lower/upper red masks

    orange_mask = cv.inRange(image, ORANGE[0], ORANGE[1])

    #get contours, then get largest contour, then get area of that largest contour for each color
    blue_contours = cv.findContours(blue_mask, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[0]
    blue = rc_utils.get_largest_contour(blue_contours, MIN_CONTOUR_AREA)
    blue_area = rc_utils.get_contour_area(blue) if blue is not None else 0

    red_contours = cv.findContours(combined_red_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]
    red = rc_utils.get_largest_contour(red_contours, MIN_CONTOUR_AREA)
    red_area = rc_utils.get_contour_area(red) if red is not None else 0

    orange_contours = cv.findContours(orange_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]
    orange = rc_utils.get_largest_contour(orange_contours, MIN_CONTOUR_AREA)
    orange_area = rc_utils.get_contour_area(orange) if orange is not None else 0
    

    #debugging
    #print('blue area: ' + str(blue_area))
    #print('red area: ' + str(red_area))
    #print('orange area: ' + str(orange_area))

#see which color is biggest, then set the contour center to whatever the biggest color's center is (same idea for area)
    if red_area >= blue_area and red_area >= orange_area:
        color = "red"
        contour_center = rc_utils.get_contour_center(red)
        contour_area = red_area
    elif blue_area >= red_area and blue_area >= orange_area:
        color = "blue"
        contour_center = rc_utils.get_contour_center(blue)
        contour_area = blue_area
    else:
        color = "orange"
        contour_center = rc_utils.get_contour_center(orange)
        contour_area = orange_area

    
    #debugging
    #print(blue_area)
    #rc.display.show_color_image(combined_red_mask)

        
        


def wall_follow():
    global speed, angle

    scan_data = rc.lidar.get_samples() #initialize scan

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

    angle = rc_utils.clamp((chosen_heading / 75 + wall_adjust) / 2.0, -1.0, 1.0) 
    #we checked heading between -75 and 75. dividing by 75 normalizes it between -1 and 1. 
    #wall angle is already clamped between -1 and 1
    #takes the average between these two values to calculate angle

    speed_threshold=400 #if best_opening is above this distance, set speed up. else, lower speed
    speed = 1.0 if best_opening > speed_threshold else 0.6 #if the path forward is very clear (the closest point is 220+ cm away), set speed = 1. if it is not so clear, set speed = 0.6

    front_distance = rc_utils.get_lidar_average_distance(scan_data,0,5)
    
    ''' uncomment irl because you can account for pitch -> this doesnt work in sim (no ros)
    if front_distance < 50: #anti-crash
        speed = -1 
'''
    #print(best_opening) 

def get_AR_marker():
    #creates markers
    global markers
    image = rc.camera.get_color_image()

    markers=rc_utils.get_ar_markers(image)

    rc_utils.draw_ar_markers(image,markers) 
    


def start():
    global speed, angle,markers
    rc.drive.set_max_speed(0.5) #for the sim
    speed = 1.0
    angle = 0.0
    rc.set_update_slow_time(0.5)

def update():
    global state, speed, angle, color, contour_area, contour_center

    scan = rc.lidar.get_samples() #set up lidar
    front_distance = rc_utils.get_lidar_average_distance(scan,0,5) #5 degree window directly in front of the car
    left_distance = rc_utils.get_lidar_average_distance(scan,-90,5) #5 degree window to the left of the car
    right_distance = rc_utils.get_lidar_average_distance(scan,90,5) #5 degree window to the right of the car

    #wall_follow()
    get_AR_marker() #ar id 1 = elevator
    area = 0 #reset area (of ar tags)

    for marker in markers: #iterate through markers

        #find marker id
        #print(marker.get_id())

        area = (marker.get_corners_aruco_format()[0][2][0] - marker.get_corners_aruco_format()[0][0][0]) * (marker.get_corners_aruco_format()[0][2][1] - marker.get_corners_aruco_format()[0][0][1]) #calculate area of ar marker
        if marker.get_id() == 1 and area > 300: #if the area is high enough (if car is close enough) and the id is right, change states
            state=1 #getting onto the elevator
            ''' tried to do some ar marker stuff here but its useless in real life because of the rainbow walls
        elif marker.get_id() == 5 and area >300: #if the area is high enough (the car is close enough), then you can change to the finishing state
            state=4
            '''
        #print(area)
    #print(area)


    if state == 0: #starting (pre-elevator)
        wall_follow() #just wall follow until the elevator
        
    elif state == 1: #getting on the elevator

        speed = 1
        angle = 0

        #reset variables
        contour_area=0
        contour_center=None

        update_contour()

        #debugging
        #print(contour_area)
        #print(color)

        
        if front_distance<300 and color == 'red': #if the car is close enough and the light is red, dont go
            speed = 0
        else: #else go towards the elevator
            speed = 0.5

        
        #debugging
        #print(front_distance)
        #print(left_distance)


        if front_distance > 70: #if it is not too close to the front, use Pcontrol to gently keep left_distance at a value (contour center was inconsistent/not working)
            setpoint = 120
            present_value = left_distance

            kp = 0.005

            error = setpoint - present_value
            angle = kp*error
            angle = rc_utils.clamp(angle,-1,1)
            #print(error)

        elif front_distance<65 and left_distance<65: #if the car is close enough and the left wall is close (means that the car is on the elevator), change states 
            state = 2

    elif state == 2: #getting off the elevator
        if left_distance>60:
            #if the left distance goes up -> elevator is at the top, change states
            state=3
    elif state == 3: #past the elevator
        speed=1 #get off the elevator
        
        
        #if both green and blue are in frame, set setpoint to the left side of the screen -> force turn left (optimal line is two left turns after the elevator)
        image = rc.camera.get_color_image()

        image = rc_utils.crop(image, (0,0),(rc.camera.get_height(),200))
        image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        
        green_mask = cv.inRange(image,SIMGREEN[0],SIMGREEN[1])
        blue_mask = cv.inRange(image,BLUE[0],BLUE[1])

        green_contours,_ = cv.findContours(green_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        blue_contours,_ = cv.findContours(blue_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        #debugging
        #rc.display.show_color_image(green_mask)
        #print(green_contours)

        #pcontrol
        if green_contours: 
            green = rc_utils.get_largest_contour(green_contours)
            if green is not None:
                contour_center = rc_utils.get_contour_center(green)
                setpoint = rc.camera.get_width()
                present_value = contour_center[1]

                kp = -0.005

                error = setpoint - present_value
                angle = kp*error
                angle = rc_utils.clamp(angle,-1,1) 
        elif blue_contours:
            blue = rc_utils.get_largest_contour(blue_contours)
            if blue is not None:
                contour_center = rc_utils.get_contour_center(blue)
                setpoint = rc.camera.get_width()
                present_value = contour_center[1]

                kp = -0.005

                error = setpoint - present_value
                angle = kp* error
                angle = rc_utils.clamp(angle,-1,1)
        else:
            #after we turn left, we have no reason to line follow since wall follow will work 
            #however, in case the camera temporarily misses both the green and blue, I wanted to ensure the car doesn't start wall following too early
            wall_follow()
            

        
    rc.drive.set_speed_angle(speed, angle) 
        
    

    

def update_slow():
    global state
    print(state)
    for marker in markers:
        #print(f"Marker ID: {marker.get_id()}")
        #print(f"Marker Corners: {marker.get_corners_aruco_format()}")
        #print(f"Marker Orientation: {marker.get_orientation()}")
        pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
