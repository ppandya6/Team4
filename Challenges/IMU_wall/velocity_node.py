# velocity_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class VelocityNode(Node):
    def __init__(self):
        super().__init__('velocity_node')
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.pub_vel = self.create_publisher(Twist, '/velocity', 10)

        self.prev_t = self.get_clock().now()
        # keep velocity state in m/s
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

    def imu_cb(self, msg: Imu):
        now = self.get_clock().now()
        dt = (now - self.prev_t).nanoseconds * 1e-9
        self.prev_t = now

        # accel in body frame (m/s²)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # simple integration
        self.vx += ax * dt
        self.vy += ay * dt
        self.vz += az * dt

        # publish as Twist (we only care about linear.x/y here)
        vel = Twist()
        vel.linear.x = self.vx
        vel.linear.y = self.vy
        vel.linear.z = self.vz
        self.pub_vel.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
