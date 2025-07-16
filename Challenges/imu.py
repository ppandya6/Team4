# TODO: Implement imu
# Template
"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-summer-labs

File Name: lab_2a.py

Title: BYOA (Build Your Own AHRS)

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: The goal of this lab is to build and deploy a ROS node that can ingest
IMU data and return accurate attitude estimates (roll, pitch, yaw) that can then
be used for autonomous navigation. It is recommended to review the equations of
motion and axes directions for the RACECAR Neo platform before starting. Template
code has been provided for the implementation of a Complementary Filter.

Expected Outcome: Subscribe to the /imu and /mag topics, and publish to the /attitude
topic with accurate attitude estimations.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
import numpy as np
import math
import time

class CompFilterNode(Node):
    def __init__(self):
        super().__init__('complementary_filter_node')

        # Set up subscriber and publisher nodes
        self.subscription_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.subscription_mag = self.create_subscription(MagneticField, '/mag', self.mag_callback, 10)
        self.publisher_attitude = self.create_publisher(Vector3, '/attitude', 10) # output as [roll, pitch, yaw] angles

        self.prev_time = self.get_clock().now() # initialize time checkpoint
        self.alpha = ___ # TODO: Determine an alpha value that works with the complementary filter

        # set up attitude params
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.mag = None

    # [FUNCTION] Called when new IMU data is received, attidude calc completed here as well
    def imu_callback(self, data):
        # TODO: Grab linear acceleration and angular velocity values from subscribed data points
        accel = ___
        gyro = ___

        # TODO: Calculate time delta
        now = ___ # Current ROS time
        dt = ___ # Time delta
        self.prev_time = now # refresh checkpoint

        # Attitude angle derivations, see full formula here:
        # https://ahrs.readthedocs.io/en/latest/filters/complementary.html
    
        # TODO: Derive tilt angles from accelerometer
        accel_roll = ___ # theta_x
        accel_pitch = ___ # theta_y - seems correct

        # TODO: Integrate gyroscope to get attitude angles
        gyro_roll = ___ # theta_xt
        gyro_pitch = ___ # theta_yt
        gyro_yaw = ___ # theta_zt

        # TODO: Compute yaw angle from magnetometer
        if self.mag:
            mx, my, mz = self.mag
            print(f"Mag norm (~50 uT): {math.sqrt(mx**2 + my**2 + mz**2) * 1e6}") # used for checking magnetic disturbances/offsets
            bx = ___
            by = ___
            mag_accel_yaw = ___
        else:
            mag_accel_yaw = self.yaw
        
        # TODO: Fuse gyro, mag, and accel derivations in complemtnary filter
        self.roll = ___
        self.pitch = ___
        self.yaw = ___

        # Print results for sanity checking
        print(f"====== Complementary Filter Results ======")
        print(f"Speed || Freq = {round(1/dt,0)} || dt (ms) = {round(dt*1e3, 2)}")
        print(f"Accel + Mag Derivation")
        print(f"Roll (deg): {accel_roll * 180/math.pi}")
        print(f"Pitch (deg): {accel_pitch * 180/math.pi}")
        print(f"Yaw (deg): {mag_accel_yaw * 180/math.pi}")
        print()
        print(f"Gyro Derivation")
        print(f"Roll (deg): {gyro_roll * 180/math.pi}")
        print(f"Pitch (deg): {gyro_pitch * 180/math.pi}")
        print(f"Yaw (deg): {gyro_yaw * 180/math.pi}")
        print()
        print(f"Fused Results")
        print(f"Roll (deg): {self.roll * 180/math.pi}")
        print(f"Pitch (deg): {self.pitch * 180/math.pi}")
        print(f"Yaw (deg): {self.yaw * 180/math.pi}")
        print("\n")
        
        # TODO: Publish to attitude topic (convert to degrees)
        attitude = Vector3()
        attitude.x = ___
        attitude.y = ___
        attitude.z = ___
        self.publisher_attitude.publish(attitude)
    
    # [FUNCTION] Called when magnetometer topic receives an update
    def mag_callback(self, data):
        # TODO: Assign self.mag to the magnetometer data points
        self.mag = ___

def main():
    rclpy.init(args=None)
    node = CompFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
