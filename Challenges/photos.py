"""
MIT BWSI Autonomous RACECAR
MIT License
bwsix RC101 - Fall 2023

File Name: sign_photo_capture.py

Title: Sign Photo Capture & Auto-Increment Saver

Author: <YOUR NAME OR TEAM>

Purpose:
    Capture training images of traffic signs directly from the RACECAR
    camera. Each time the operator presses the A button on the gamepad,
    the current RGB frame is saved to dataset/images/ as a JPEG.
    File names auto-increment so you never overwrite a previous shot:
        signPhoto_0001.jpg, signPhoto_0002.jpg, …

Expected Outcome:
    Press Start → the HUD shows the live camera feed
    Press A     → the frame is written to disk and a short message prints
    Hold Back   → script stops (handled by SDK)
"""

################################################################################
# Imports
################################################################################

import sys
import os
from pathlib import Path
import cv2 as cv

# If this file is nested one level deep inside labs/, adjust the relative path
# to "../library" instead of "../../library".
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

################################################################################
# Global variables
################################################################################

rc = racecar_core.create_racecar()

# Directory to store images (relative to script location)
IMG_DIR = Path(__file__).parent / "dataset/images"
IMG_DIR.mkdir(parents=True, exist_ok=True)   # ensure path exists

# Running counter for file names (starts at the next available number)
existing = sorted(IMG_DIR.glob("congaPhoto_*.jpg"))
file_counter = int(existing[-1].stem.split("_")[-1]) + 1 if existing else 1

################################################################################
# Functions
################################################################################

def start():
    pass


def update():
    """Called every frame (~60 Hz) while script is running."""
    global file_counter

    frame = rc.camera.get_color_image()
    if frame is None:
        return

    # Save image when A is pressed (debounced by SDK)
    if rc.controller.was_pressed(rc.controller.Button.A):
        filename = IMG_DIR / f"signPhoto_{file_counter:04d}.jpg"
        cv.imwrite(str(filename), frame)
        print(f"[CAPTURED] {filename.name}")
        file_counter += 1

    # Show live feed on HUD
    rc.display.show_color_image(frame)


def update_slow():
    pass

################################################################################
# DO NOT MODIFY: Register callbacks and start execution
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
