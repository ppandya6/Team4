# lab_h/cone_slalom_lidar_state.py
#
# Color-aware cone-slalom controller (camera only)

import sys, cv2 as cv, numpy as np
sys.path.insert(1, "../../library")
import racecar_core, racecar_utils as rc_utils

# ──────────────── CONFIG ────────────────
# HSV ranges for red and blue cones
RED_LO1, RED_HI1 = (0,   120,  70), (10, 255, 255)       # low red
RED_LO2, RED_HI2 = (170, 120,  70), (180,255, 255)       # high red (wrap)
BLUE_LO, BLUE_HI = (100, 150,   0), (140,255, 255)

MIN_AREA_PX   = 30        # ignore tiny blobs
CENTER_SHIFT_PX = 10     # positive value; we’ll decide sign later
DESIRED_H       = 100      # pick ~h when the cone is at the “pass” distance
PASS_H          = 150      # if h gets this big we consider the cone passed
K_STEER         = 0.7      # 4.0 was clipping to ±1 too early
K_SPEED         = 0.051     # keeps speed moderate
time = 0
queue = []  # driving queue, not used in this lab
# ──────────────── setup ────────────────
rc = racecar_core.create_racecar()

# ────────── helper: contour extractor ──────────
def find_color_contours(hsv, lo1, hi1, lo2=None, hi2=None):
    mask = cv.inRange(hsv, lo1, hi1)
    if lo2 is not None:
        mask |= cv.inRange(hsv, lo2, hi2)
    mask = cv.erode(mask, None, iterations=2)
    mask = cv.dilate(mask, None, iterations=2)
    cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    return [c for c in cnts if cv.contourArea(c) > MIN_AREA_PX]

# ─────────── lifecycle callbacks ───────────
def start():
    print("Cone-slalom controller ready.")

def update():
    global time, K_SPEED, K_STEER, RED_LO1, RED_HI1, RED_LO2, RED_HI2, BLUE_LO, BLUE_HI, MIN_AREA_PX, CENTER_SHIFT_PX, DESIRED_H, PASS_H
    frame = rc.camera.get_color_image()
    if frame is None:
        return
    
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # ---------- masks ----------
    mask_red  = cv.inRange(hsv, RED_LO1, RED_HI1) | cv.inRange(hsv, RED_LO2, RED_HI2)
    mask_blue = cv.inRange(hsv, BLUE_LO,  BLUE_HI)

    # ---------- find contours ----------
    reds  = find_color_contours(hsv, RED_LO1, RED_HI1, RED_LO2, RED_HI2)
    blues = find_color_contours(hsv, BLUE_LO,  BLUE_HI)

    # ---------- pick target ----------
    if reds:
        target, color = max(reds, key=cv.contourArea), "red"
        
    elif blues:
        target, color = max(blues, key=cv.contourArea), "blue"
    else:
        rc.drive.stop()
        #print("No cones.")
        return

    # ---------- geometry ----------
    # --- geometry (add imgW!) ---
    x, y, w, h = cv.boundingRect(target)
    print("H: ", h)
    cx  = x + w/2
    imgW = rc.camera.get_width()          # you were missing this line

    # --- lateral set-point (shift right for red, left for blue) ---
    center_shift = +CENTER_SHIFT_PX if color == "red" else -CENTER_SHIFT_PX
    setpt_x      = imgW/2 + center_shift

    # --- steering error: POSITIVE means "steer left" ---
    steer_err = (setpt_x - cx) / (imgW/2)

    # --- range & speed ---
    range_err = max(0, DESIRED_H - h)
    
    speed     = np.clip(K_SPEED*range_err, 0, 1)   # never 0 so it keeps moving
    steering  = np.clip(K_STEER * steer_err, -1, 1)
    if color == "blue":
        speed *= 0.7
        steering *= 1.4

    steering  = np.clip(K_STEER * steer_err, -1, 1)
    print("Steering:", steering, "Speed:", speed, "Color:", color, "Steer Err:", steer_err)
    


    # ---------- VISUAL DEBUG ----------
    debug = frame.copy()
    # draw bounding box & centroid
    cv.rectangle(debug, (x, y), (x+w, y+h), (0,0,255) if color=="red" else (255,0,0), 2)
    cv.circle(debug, (int(cx), y+h//2), 4, (0,255,0), -1)
    # draw original centre line and shifted set-point
    cv.line(debug, (imgW//2, 0), (imgW//2, frame.shape[0]), (200,200,200), 1)
    cv.line(debug,
            (imgW//2 + center_shift, 0),
            (imgW//2 + center_shift, frame.shape[0]),
            (0,255,255), 1)
    # text overlay
    cv.putText(debug,
               f"{color} h={h} steer={steering:+.2f} speed={speed:+.2f}",
               (10,25), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    cv.imshow("view", debug)
    cv.imshow("mask red",  mask_red)
    cv.imshow("mask blue", mask_blue)
    cv.waitKey(1)          # required for imshow to refresh

    # ---------- pass-cone reset ----------
    if len(queue) > 0:
        # Pull speed and angle from the first entry
        speed = queue[0][1]         # ← index 1 is speed
        steering = queue[0][2]         # ← index 2 is angle (normalized -1.0 to 1.0)
        # Decrease its remaining time by the elapsed frame time
        queue[0][0] -= rc.get_delta_time()  # ← decrement time_remaining
        # If time has elapsed, remove this instruction
        if queue[0][0] <= 0:
            queue.pop(0)
    if h > PASS_H:
         if color == "red":
            queue.append([1.1, 1, 0])
            queue.append([0.2, 1, -1])
         else:
            blues.clear()
    rc.drive.set_speed_angle(speed, steering)

def update_slow():
    pass

# ──────────────── run ────────────────
if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
