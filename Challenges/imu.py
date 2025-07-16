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
        self.alpha = 0.98  # Typical value between 0.95–0.99 for complementary filter

        # set up attitude params
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.mag = None

    def imu_callback(self, data):
        # Grab linear acceleration and angular velocity values
        accel = data.linear_acceleration
        gyro = data.angular_velocity

        # Calculate time delta
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9  # Convert nanoseconds to seconds
        self.prev_time = now

        # Derive tilt angles from accelerometer
        accel_roll = math.atan2(accel.y, accel.z)
        accel_pitch = math.atan2(-accel.x, math.sqrt(accel.y**2 + accel.z**2))

        # Integrate gyroscope to get attitude angles
        gyro_roll = self.roll + gyro.x * dt
        gyro_pitch = self.pitch + gyro.y * dt
        gyro_yaw = self.yaw + gyro.z * dt

        # Compute yaw angle from magnetometer
        if self.mag:
            mx, my, mz = self.mag
            print(f"Mag norm (~50 uT): {math.sqrt(mx**2 + my**2 + mz**2) * 1e6}")  # diagnostic
            bx = mx * math.cos(accel_pitch) + mz * math.sin(accel_pitch)
            by = mx * math.sin(accel_roll) * math.sin(accel_pitch) + my * math.cos(accel_roll) - mz * math.sin(accel_roll) * math.cos(accel_pitch)
            mag_accel_yaw = math.atan2(-by, bx)
        else:
            mag_accel_yaw = self.yaw

        # Fuse gyro and accel (and mag for yaw) in complementary filter
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        self.yaw = self.alpha * gyro_yaw + (1 - self.alpha) * mag_accel_yaw

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

        # Publish to attitude topic (convert to degrees)
        attitude = Vector3()
        attitude.x = self.roll * 180.0 / math.pi
        attitude.y = self.pitch * 180.0 / math.pi
        attitude.z = self.yaw * 180.0 / math.pi
        self.publisher_attitude.publish(attitude)

    def mag_callback(self, data):
        # Assign self.mag to the magnetometer data points
        self.mag = (data.magnetic_field.x, data.magnetic_field.y, data.magnetic_field.z)

def main():
    rclpy.init(args=None)
    node = CompFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
