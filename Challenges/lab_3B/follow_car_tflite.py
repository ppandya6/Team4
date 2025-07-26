from pycoral.utils.edgetpu import run_inference, make_interpreter
from pycoral.utils.dataset import read_label_file
from pycoral.adapters.detect import get_objects
from pycoral.adapters.common import input_size
import sys
import cv2
import numpy as np

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
kp = 0.00511
kd = 0.0036

# Variables for PD control
prev_error = 0
prev_time = 0
target_car = None

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
    global prev_error, prev_time, target_car

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

    # Assume first object is the one to follow
    target_car = objs[0]
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
    angle = kp * error + kd * derivative
    angle = rc_utils.clamp(angle, -1, 1)

    # Constant speed
    speed = 0.85

    # Drive
    rc.drive.set_speed_angle(speed, angle)

    # Update variables
    prev_error = error
    prev_time = current_time

def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
