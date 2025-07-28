from pycoral.utils.edgetpu import run_inference, make_interpreter
from pycoral.utils.dataset import read_label_file
from pycoral.adapters.detect import get_objects
from pycoral.adapters.common import input_size
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math


sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

# Model and label paths
model_path = "congaf_edgetpu.tflite"
label_path = "labels_conga.txt"

# Interpreter setup
interpreter = None
inference_size = (0, 0)
labels = ()

# PD control parameters (initially tuned; adjust as needed)
kp = 0.0022
ku = 0.0025
kd = 0.1 * kp

# Variables for PD control
prev_error = 0
prev_time = 0
target_car = None
elapsed_time = 0
position = []
time = []

def start():
    global interpreter, inference_size, labels, prev_time
    rc.drive.set_speed_angle(0, 0)

    try:
        interpreter = make_interpreter(model_path)
        interpreter.allocate_tensors()
        labels = read_label_file(label_path)
        inference_size = input_size(interpreter)
        print(f"Model {model_path} loaded")
    except (ValueError, RuntimeError) as e:
        print(f"Error loading model: {e}")
        exit()

    prev_time = rc.get_delta_time()
    print("Camera-based wall follower initialized.")

def update():
    kp = 0.0022
    ku = 0.0025
    kd = 0.1 * kp
    global prev_error, prev_time, target_car, elapsed_time, position, time

    image = rc.camera.get_color_image()
    if image is None:
        rc.drive.set_speed_angle(0, 0)
        print("No camera image.")
        return

    # Preprocess image for inference
    image_inference = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_inference = cv2.resize(image_inference, inference_size)

    run_inference(interpreter, image_inference.tobytes())
    objs = get_objects(interpreter, 0.1)

    if not objs:
        rc.drive.set_speed_angle(0, 0)
        print("Target not detected")
        return
    else:
        print("Car seen")

    



    # Assume first object is the one to follow
    target_car = objs[0]
    print(target_car)
    bbox = target_car.bbox
    center_x = (bbox.xmin + bbox.xmax) // 2

    # Calculate error
    frame_center_x = inference_size[0] // 2
    error = center_x - frame_center_x

    # Time difference
    current_time = rc.get_delta_time()
    dt = current_time - prev_time
    dt = dt if dt > 0 else 1e-5  # prevent division by zero

    # PD control
    derivative = (error - prev_error) / dt
    angle = (kp * error + kd * derivative)
    angle = rc_utils.clamp(angle, -1, 1)

    forward_angle = 60 #angle to find distance
    sample_window = 2 #window size
    #average distance at 60 degrees left/right w/ 2 degree window (we checked this angle for wall follow)
    kp = 0.003 
    scan = rc.lidar.get_samples()
    
    _, closest_right = rc_utils.get_lidar_closest_point(scan, (0, 180)) #closest point on the right half
    _, closest_left = rc_utils.get_lidar_closest_point(scan, (180, 360)) #closest point on the left half
    right_forward = rc_utils.get_lidar_average_distance(scan, forward_angle, sample_window) #average distance at 60 degrees left w/ 2 degree window
    left_forward = rc_utils.get_lidar_average_distance(scan, 360 - forward_angle, sample_window) #average distance at 60 degrees left w/ 2 degree window
    
    #b = sqrt(a**2 - c**2)
    right_wall_length = math.sqrt(max(0, right_forward ** 2 - closest_right** 2)) #pythagorean theorem (avoids complex numbers with max())
    left_wall_length = math.sqrt(max(0, left_forward ** 2 - closest_left ** 2)) #pythagorean theorem (avoids complex numbers with max())

    #if the wall length gets bigger, then the wall is either closer or slanting towards the car

    error = right_wall_length - left_wall_length #if error = 0, then the right and left wall lengths are equal, meaning the car is centered
    wall_adjust = rc_utils.clamp(error * kp, -1, 1) #clamps

    merged_angle = rc_utils.clamp((angle + wall_adjust) / 2.0, -1.0, 1.0) 
    #we checked heading between -75 and 75. dividing by 75 normalizes it between -1 and 1.
    # Constant speed
    
    speed = 0.6

    # Drive
    rc.drive.set_speed_angle(speed, merged_angle)

    # Update variables
    prev_error = error
    prev_time = current_time  
    elapsed_time += rc.get_delta_time()
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
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
