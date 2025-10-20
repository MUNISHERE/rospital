import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import numpy as np
from .planner_wrapper import NeuPANWrapper

class NeuPANNode(Node):
    def __init__(self):
        super().__init__('neupan_node')

        # Load parameters
        self.declare_parameters(namespace='', parameters=[
            ('planner.model_path', ''),
            ('planner.kinematics', 'diff'),
            ('planner.control_rate', 10.0),
            ('planner.cmd_topic', '/cmd_vel'),
            ('planner.laser_topic', '/scan'),
            ('planner.odom_topic', '/odom'),
            ('planner.path_topic', '/initial_path')
        ])
        self.model_path = self.get_parameter('planner.model_path').get_parameter_value().string_value
        self.rate = self.get_parameter('planner.control_rate').get_parameter_value().double_value

        # Init planner
        self.planner = NeuPANWrapper(self.model_path)

        # Subscribers
        self.create_subscription(LaserScan, self.get_parameter('planner.laser_topic').value, self.laser_callback, 10)
        self.create_subscription(Odometry, self.get_parameter('planner.odom_topic').value, self.odom_callback, 10)
        self.create_subscription(Path, self.get_parameter('planner.path_topic').value, self.path_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('planner.cmd_topic').value, 10)

        # Data buffers
        self.latest_scan = None
        self.latest_pose = None
        self.latest_path = None

        # Timer
        self.create_timer(1.0 / self.rate, self.control_loop)

    def laser_callback(self, msg):
        self.latest_scan = np.array(msg.ranges)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # convert quaternion -> yaw
        import math
        import tf_transformations
        euler = tf_transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.latest_pose = [pos.x, pos.y, euler[2]]

    def path_callback(self, msg):
        self.latest_path = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]

    def control_loop(self):
        if self.latest_scan is None or self.latest_pose is None or self.latest_path is None:
            return
        v, w = self.planner.compute_cmd(self.latest_scan, self.latest_pose, self.latest_path)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NeuPANNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
