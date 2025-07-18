# pose_estimate_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, Vector3

class PoseEstimateNode(Node):
    def __init__(self):
        super().__init__('pose_estimate_node')
        self.sub_vel = self.create_subscription(Twist, '/velocity', self.vel_cb, 10)
        self.sub_att = self.create_subscription(Vector3, '/attitude', self.att_cb, 10)
        self.pub_pose = self.create_publisher(Pose2D, '/pose_estimate', 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # yaw in radians

        self.vx = 0.0
        self.vy = 0.0
        self.prev_t = self.get_clock().now()

    def vel_cb(self, vel: Twist):
        now = self.get_clock().now()
        dt = (now - self.prev_t).nanoseconds * 1e-9
        self.prev_t = now

        # get last-attitude θ
        # integrate forward in global frame
        dx = (vel.linear.x * cos(self.theta) - vel.linear.y * sin(self.theta)) * dt
        dy = (vel.linear.x * sin(self.theta) + vel.linear.y * cos(self.theta)) * dt

        self.x += dx
        self.y += dy

        # publish
        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.theta
        self.pub_pose.publish(pose)

    def att_cb(self, att: Vector3):
        # Vector3.z is yaw in degrees → convert to radians
        import math
        self.theta = math.radians(att.z)

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
