import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleSim(Node):
    def __init__(self):
        super().__init__('turtle_sim_web')

        self.pose = Pose(x=5.5, y=5.5, theta=0.0)
        self.v = 0.0
        self.w = 0.0

        self.pose_pub = self.create_publisher(
            Pose, '/turtle1/pose', 10
        )

        self.cmd_sub = self.create_subscription(
            Twist, '/turtle1/cmd_vel', self.cmd_cb, 10
        )

        self.timer = self.create_timer(0.05, self.update)

    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        dt = 0.05
        self.pose.theta += self.w * dt
        self.pose.x += self.v * math.cos(self.pose.theta) * dt
        self.pose.y += self.v * math.sin(self.pose.theta) * dt
        self.pose_pub.publish(self.pose)

def main():
    rclpy.init()
    node = TurtleSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
