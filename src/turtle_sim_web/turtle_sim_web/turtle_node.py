import math
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import SetBool

class TurtleSim(Node):
    def __init__(self):
        super().__init__('turtle_sim_web')
        self.teleport_count = 0

        self.pose = Pose(x=5.5, y=5.5, theta=0.0)
        self.v = 0.0
        self.w = 0.0

        self.pose_pub = self.create_publisher(Pose, '/turtle1/pose', 10)
        self.cmd_sub = self.create_subscription(Twist, '/turtle1/cmd_vel', self.cmd_cb, 10)

        # Teleport service - FIXED
        self.cb_group = ReentrantCallbackGroup()
        self.teleport_srv = self.create_service(
            SetBool, 
            'teleport_turtle', 
            self.teleport_cb, 
            callback_group=self.cb_group
        )

        self.timer = self.create_timer(0.05, self.update)

    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def teleport_cb(self, request, response):
        positions = [(1.0,10.0), (10.0,10.0), (1.0,1.0), (10.0,1.0)]
        self.teleport_count += 1
        idx = (self.teleport_count - 1) % 4
        self.pose.x, self.pose.y = positions[idx]
        self.pose.theta = 0.0
        self.pose_pub.publish(self.pose)
        response.success = True
        response.message = f"Teleported to corner {idx}"
        self.get_logger().info(f'Teleported to corner {idx}: ({self.pose.x:.1f}, {self.pose.y:.1f})')
        return response

    def update(self):
        dt = 0.05
        self.pose.theta += self.w * dt
        self.pose.x += self.v * math.cos(self.pose.theta) * dt
        self.pose.y += self.v * math.sin(self.pose.theta) * dt
        self.pose_pub.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSim()
    node.get_logger().info
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
