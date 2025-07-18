"""
Copyright MIT
MIT License

BWSI Autonomous RACECAR Course
Racecar Neo LTS

File Name: physics_real.py
File Description: Contains the Physics module of the racecar_core library
"""

from physics import Physics

# General
from collections import deque
import numpy as np
from nptyping import NDArray
from geometry_msgs.msg import Vector3, Twist, Pose2D

# ROS2
import rclpy as ros2
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSProfile,
)
from sensor_msgs.msg import Imu, MagneticField


class PhysicsReal(Physics):
    # Topics
    __IMU_TOPIC     = "/imu"
    __MAG_TOPIC     = "/mag"
    __ATTITUDE      = "/attitude"
    __VELOCITY      = "/velocity"
    __POSE_ESTIMATE = "/pose_estimate"

    # Limit on buffer size to prevent memory overflow
    __BUFFER_CAP = 60

    def __init__(self):
        self.node = ros2.create_node("imu_sub")

        # QoS
        qos = QoSProfile(depth=1)
        qos.history    = QoSHistoryPolicy.KEEP_LAST
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.durability  = QoSDurabilityPolicy.VOLATILE

        # Subscriptions
        self.__imu_sub      = self.node.create_subscription(Imu,           self.__IMU_TOPIC,     self.__imu_callback,      qos)
        self.__mag_sub      = self.node.create_subscription(MagneticField, self.__MAG_TOPIC,     self.__mag_callback,      qos)
        self.__att_sub      = self.node.create_subscription(Vector3,       self.__ATTITUDE,      self.__attitude_callback, qos)
        self.__vel_sub      = self.node.create_subscription(Twist,         self.__VELOCITY,      self.__velocity_callback, qos)
        self.__pose_sub     = self.node.create_subscription(Pose2D,        self.__POSE_ESTIMATE, self.__pose_callback,     qos)

        # Buffers & state
        self.__acceleration          = np.zeros(3)
        self.__acceleration_buffer   = deque()
        self.__angular_velocity      = np.zeros(3)
        self.__angular_velocity_buffer = deque()
        self.__magnetic_field        = np.zeros(3)
        self.__magnetic_field_buffer = deque()
        self.__attitude              = np.zeros(3)
        self.__attitude_buffer       = deque()
        self.__velocity              = np.zeros(3)
        self.__velocity_buffer       = deque()
        self.__pose                  = np.zeros(3)  # [x, y, theta]
        self.__pose_buffer           = deque()

    # callbacks just push into their deques:
    def __imu_callback(self, data):
        self.__acceleration_buffer.append(
            np.array([data.linear_acceleration.x,
                      data.linear_acceleration.y,
                      data.linear_acceleration.z])
        )
        self.__angular_velocity_buffer.append(
            np.array([data.angular_velocity.x,
                      data.angular_velocity.y,
                      data.angular_velocity.z])
        )
        # drop old
        if len(self.__acceleration_buffer)    > self.__BUFFER_CAP: self.__acceleration_buffer.popleft()
        if len(self.__angular_velocity_buffer)> self.__BUFFER_CAP: self.__angular_velocity_buffer.popleft()

    def __mag_callback(self, data):
        self.__magnetic_field_buffer.append(
            np.array([data.magnetic_field.x,
                      data.magnetic_field.y,
                      data.magnetic_field.z])
        )
        if len(self.__magnetic_field_buffer) > self.__BUFFER_CAP:
            self.__magnetic_field_buffer.popleft()

    def __attitude_callback(self, data):
        self.__attitude_buffer.append(np.array([data.x, data.y, data.z]))
        if len(self.__attitude_buffer) > self.__BUFFER_CAP:
            self.__attitude_buffer.popleft()

    def __velocity_callback(self, data):
        # Twist: linear.x/y, angular.z
        self.__velocity_buffer.append(
            np.array([data.linear.x, data.linear.y, data.angular.z])
        )
        if len(self.__velocity_buffer) > self.__BUFFER_CAP:
            self.__velocity_buffer.popleft()

    def __pose_callback(self, data):
        # Pose2D: x, y, theta
        self.__pose_buffer.append(
            np.array([data.x, data.y, data.theta])
        )
        if len(self.__pose_buffer) > self.__BUFFER_CAP:
            self.__pose_buffer.popleft()

    # average & clear any buffers that have new data
    def __update(self):
        if self.__acceleration_buffer:
            self.__acceleration = np.mean(self.__acceleration_buffer, axis=0)
            self.__acceleration_buffer.clear()

        if self.__angular_velocity_buffer:
            self.__angular_velocity = np.mean(self.__angular_velocity_buffer, axis=0)
            self.__angular_velocity_buffer.clear()

        if self.__magnetic_field_buffer:
            self.__magnetic_field = np.mean(self.__magnetic_field_buffer, axis=0)
            self.__magnetic_field_buffer.clear()

        if self.__attitude_buffer:
            self.__attitude = np.mean(self.__attitude_buffer, axis=0)
            self.__attitude_buffer.clear()

        if self.__velocity_buffer:
            self.__velocity = np.mean(self.__velocity_buffer, axis=0)
            self.__velocity_buffer.clear()

        if self.__pose_buffer:
            self.__pose = np.mean(self.__pose_buffer, axis=0)
            self.__pose_buffer.clear()

    # getters
    def get_linear_acceleration(self) -> NDArray[3, np.float32]:
        self.__update()
        return np.array(self.__acceleration, dtype=np.float32)

    def get_angular_velocity(self) -> NDArray[3, np.float32]:
        self.__update()
        return np.array(self.__angular_velocity, dtype=np.float32)

    def get_magnetic_field(self) -> NDArray[3, np.float32]:
        self.__update()
        return np.array(self.__magnetic_field, dtype=np.float32)

    def get_attitude(self) -> NDArray[3, np.float32]:
        self.__update()
        return np.array(self.__attitude, dtype=np.float32)

    def get_velocity(self) -> NDArray[3, np.float32]:
        """
        Returns [vx, vy, ωz] as buffered from /velocity
        """
        self.__update()
        return np.array(self.__velocity, dtype=np.float32)

    def get_pose_estimate(self) -> NDArray[3, np.float32]:
        """
        Returns [x, y, θ] as buffered from /pose_estimate
        """
        self.__update()
        return np.array(self.__pose, dtype=np.float32)
