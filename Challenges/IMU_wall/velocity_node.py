# velocity_kf_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class VelocityKFNode(Node):
    def __init__(self):
        super().__init__('velocity_kf_node')

        # 1) ROS interfaces
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.pub_vel = self.create_publisher(Twist, '/velocity', 10)

        # 2) Timing
        self.prev_t = self.get_clock().now()

        # 3) Kalman state for vx and vy
        self.vx = 0.0       # state estimate
        self.vy = 0.0
        self.Px = 1.0       # estimate covariance
        self.Py = 1.0

        # 4) Tuning parameters
        self.Q = 0.01        # process noise (how much we trust the model)
        self.R = 2.0        # measurement noise (how noisy is our “raw” integrated vel)

    def imu_cb(self, msg: Imu):
        # a) compute dt in seconds
        now = self.get_clock().now()
        dt  = (now - self.prev_t).nanoseconds * 1e-9
        self.prev_t = now

        # b) read acceleration
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        # ——————————————————————————
        # Prediction step (constant-velocity model)
        # v_pred = v_prev + 0*dt  (we assume velocity doesn’t change without control)
        # P_pred = P_prev + Q
        self.Px += self.Q
        self.Py += self.Q

        # c) get “measurement” by raw integration
        vx_meas = self.vx + ax * dt
        vy_meas = self.vy + ay * dt
        print(vx_meas, vy_meas)
        if vx_meas > 1 or vy_meas > 1:
            vx_meas = 0
            vy_meas = 0
        # Update step (Kalman gain & state update)
        Kx = self.Px / (self.Px + self.R)
        Ky = self.Py / (self.Py + self.R)

        self.vx = self.vx + Kx * (vx_meas - self.vx)
        self.vy = self.vy + Ky * (vy_meas - self.vy)

        # update covariances
        self.Px = (1 - Kx) * self.Px
        self.Py = (1 - Ky) * self.Py
        # ——————————————————————————

        # d) publish the filtered velocity
        vel = Twist()
        vel.linear.x = float(self.vx)
        vel.linear.y = float(self.vy)
        vel.linear.z = 0.0
        self.pub_vel.publish(vel)

        

def main(args=None):
    rclpy.init(args=args)
    node = VelocityKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
