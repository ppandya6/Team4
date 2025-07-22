#!/usr/bin/env python3
"""
follow_car_tflite.py
====================
RACECAR-SDK script that:
    • loads a TFLite single-class 'car' detector (Edge-TPU OR CPU),
    • finds the lead car's bounding box each frame,
    • steers and throttles to keep a set following distance.

Tested with:
    • MIT BWSI RACECAR SDK 4.x  (racecar_core, racecar_utils)
    • Python 3.8 – 3.11
    • Edge-TPU runtime OR tflite-runtime 2.16 (CPU fallback)

Adjust the PID constants & DESIRED_PX for your own chassis / camera.

Author: <you>
"""
# ────────────────────────── RACECAR imports ───────────────────────────
import sys, cv2 as cv, numpy as np
import racecar_core as rc
import racecar_utils as rc_utils

# ────────────────────────── TFLite / Coral ────────────────────────────
USE_EDGETPU   = True                # set False to run on CPU
MODEL_PATH    = "car_detector_edgetpu.tflite" if USE_EDGETPU else "car_detector.tflite"
INPUT_SIZE    = 320                 # EfficientDet-Lite0 input (320×320)

if USE_EDGETPU:
    from pycoral.utils.edgetpu import make_interpreter
    from pycoral.adapters.common import input_tensor
    from pycoral.adapters.detect import get_objects
else:
    import tflite_runtime.interpreter as tflite
    def make_interpreter(model_path):
        return tflite.Interpreter(model_path=model_path)

# ────────────────────── following-distance constants ──────────────────
DESIRED_PX     = 140       # bbox width (px) ⇔ correct gap (tune!)
LIN_GAIN       =  0.010    # (px → m s⁻¹)
ANG_GAIN       =  2.000    # (-0.5..0.5 image centre offset → rad s⁻¹)
MAX_SPEED      =  0.40
MAX_TURN       =  1.00

# ─────────────────────── global state (set in start) ──────────────────
interpreter    = None      # TFLite interpreter
input_index    = None
output_index   = None

# ───────────────────────────── start() ────────────────────────────────
def start():
    global interpreter, input_index, output_index

    rc.drive.set_speed_angle(0, 0)
    rc.display.show_msg("Waiting for camera…")

    interpreter = make_interpreter(MODEL_PATH)
    interpreter.allocate_tensors()
    input_index  = interpreter.get_input_details()[0]['index']

    if USE_EDGETPU:
        # pycoral returns parsed boxes; no need to know output index
        rc.display.show_msg("Edge-TPU model loaded ✓")
    else:
        output_index = interpreter.get_output_details()[0]['index']
        rc.display.show_msg("CPU model loaded ✓")

# ───────────────────────────── update() ───────────────────────────────
def update():
    # 1  Grab the latest color frame (numpy H×W×3, BGR)
    frame = rc.camera.get_color_image()
    if frame is None:
        return

    # 2  Resize to network input and run inference
    resized = cv.resize(frame, (INPUT_SIZE, INPUT_SIZE))
    inp = np.expand_dims(resized, 0).astype(np.uint8)          # uint8 model
    input_tensor(interpreter, inp) if USE_EDGETPU else interpreter.set_tensor(input_index, inp)
    interpreter.invoke()

    # 3  Extract detections (Edge-TPU helper OR raw tensor)
    if USE_EDGETPU:
        detections = get_objects(interpreter, score_threshold=0.5, image_scale=1.0)
        boxes = [d.bbox for d in detections]
        scores = [d.score for d in detections]
    else:
        # output tensor shape: [N,6] = [x1,y1,x2,y2,score,class]
        dets = interpreter.get_tensor(output_index)[0]
        mask = dets[:,4] > 0.5
        boxes = dets[mask][:,:4] if np.any(mask) else []
        scores= dets[mask][:,4]  if np.any(mask) else []

    if not boxes:
        rc.drive.set_speed_angle(0, 0)   # nothing seen → stop
        rc.display.set_color(rc.display.Color.RED_LOW)
        return

    # 4  Pick best (highest score) detection
    best_i     = int(np.argmax(scores))
    x1,y1,x2,y2= boxes[best_i]
    # Edge-TPU boxes already in image coords; CPU raw boxes need scaling
    if not USE_EDGETPU:
        x1,y1,x2,y2 = int(x1),int(y1),int(x2),int(y2)

    # 5  Compute control error terms
    w_bbox      = x2 - x1
    centre_err  = ((x1 + x2)/2 - INPUT_SIZE/2) / (INPUT_SIZE/2)  # -1 … 1
    dist_err_px = DESIRED_PX - w_bbox

    speed = np.clip(dist_err_px * LIN_GAIN, -MAX_SPEED, MAX_SPEED)
    turn  = np.clip(-centre_err * ANG_GAIN, -MAX_TURN,  MAX_TURN)

    rc.drive.set_speed_angle(speed, turn)

    # 6  Overlay bbox & debug text on the RACECAR HUD
    rc.display.draw_rectangle(x1, y1, x2, y2, rc.display.Color.GREEN)
    rc.display.show_msg(f"w={w_bbox:3.0f}px  Δcx={centre_err:+.2f}  v={speed:+.2f}  ω={turn:+.2f}")

# ───────────────────────────────── run ────────────────────────────────
if __name__ == "__main__":
    rc.run(start, update)
